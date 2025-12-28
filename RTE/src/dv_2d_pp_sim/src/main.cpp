#include <filesystem>
#include <iostream>
#include <thread>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/subscriber.h>

#include "GUI/gui.h"
#include "GUI/ImGuiExtensions.h"
#include "../include/External/imgui_internal.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>

#include "ConfigHelper.h"
#include "../include/Simulation.h"
#include "dv_interfaces/Cones.h"

// TODO:
//  + Import / Export settings and the Track
//  - Smart cone density in corners

#define ADD_ON_TRACK_DIST_THRESHOLD 1

static Simulation *sim = nullptr;

void fakeConeCallback(const dv_interfaces::Cones &cones_msg) {
    printf("Got a callback!\nsize:%lu\n", cones_msg.cones.size());
}

int OnGui() {
    ImGuiStyle &style = ImGui::GetStyle();
    static Vec2 canvas_offset{200, 300}, canvas_zoom{10, 10};
    auto all_avail = ImGui::GetContentRegionAvail();
    static bool adding_points = false;
    static bool show_cones = true, show_track_points = true, show_track_center_line = true, show_pp_points = true,
            show_car = true, show_pp_markers = false;
    static bool shuffle_cones = false;
    static int cones_to_send = -1;

    // -------------------------- LEFT SIDE CONTROL WINDOW --------------------------
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar;
    constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeX;
    ImGui::SetNextWindowSizeConstraints({200, -1}, {all_avail.x - 400, -1});
    if (ImGui::BeginChild("LeftControl", {350, -1}, child_flags, window_flags)) {
        if (ImGui::CollapsingHeader("Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Add Points", &adding_points);
            if (ImGui::Checkbox("Smooth Curve", &sim->smooth_curve)) {
                sim->Recalculate_Bezier();
                sim->Recalculate_Cones();
            }
            if (ImGui::Checkbox("Close Loop", &sim->close_loop)) {
                sim->Recalculate_Bezier();
                sim->Recalculate_Cones();
            }
        }

        if (ImGui::Button("Clear Track", {-1, 0})) {
            ImGui::OpenPopup("Delete Track?##DeleteTrackConfirmation");
        }

        if (ImGui::BeginPopupModal("Delete Track?##DeleteTrackConfirmation", nullptr,
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
            ImGui::Text("Are you sure you want to delete the track?");
            ImGui::Separator();
            auto popupAvail = ImGui::GetContentRegionAvail();
            popupAvail.x -= style.ItemSpacing.x;
            if (ImGui::Button("Yes", {popupAvail.x/2, 0})) {
                sim->center_points.clear();
                sim->bezier_control_points.clear();
                sim->cones_left_positions.clear();
                sim->cones_right_positions.clear();
                sim->Recalculate_Bezier();
                sim->Recalculate_Cones();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("No", {popupAvail.x/2, 0})) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::Separator();

        if (ImGui::CollapsingHeader("Cones Generation Settings", ImGuiTreeNodeFlags_None)) {
            if (ImGui::SliderFloat("Max. Cone Dist.", &sim->cones_max_distance, 0, 10)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Min. Cone Dist.", &sim->cones_min_distance, 0.05, 10)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Padding", &sim->cones_padding, 0.05, 10)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Dist. Variance", &sim->cones_rand_distance, 0, 1)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Padd. Variance", &sim->cones_rand_padding, 0, 1)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Kicked chance", &sim->cones_kicked_percentage, 0, 1)) {
                sim->Recalculate_Cones();
            }
            if (ImGui::SliderFloat("Kicked strength", &sim->cones_kick_strength, 0, 2)) {
                sim->Recalculate_Cones();
            }
        }

        ImGui::Separator();

        // Visibility Settings
        if (ImGui::CollapsingHeader("Visibility Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Track Points", &show_track_points);
            ImGui::Checkbox("Track Center Line", &show_track_center_line);
            ImGui::Checkbox("Vehicle", &show_car);
            ImGui::Checkbox("Cones", &show_cones);
            ImGui::Checkbox("Path Planning Points", &show_pp_points);
            ImGui::Checkbox("Path Planning Markers", &show_pp_markers);

            if (ImGui::Button("Reset Path Planning Markers")) {
                sim->ResetPathPlanningMarkers();
            }
        }

        ImGui::Separator();

        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Shuffle Cones", &shuffle_cones);
            ImGui::SliderInt("Cones to send", &cones_to_send, -1, std::max(sim->cones_left_positions.size(), sim->cones_right_positions.size()));
            if (ImGui::BeginTable("##PathPlanningUpdateButtons", 2,
                      ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                ImGui::BeginDisabled(sim->bezier_control_points.size() < (sim->smooth_curve ? 2 : 1));
                if (ImGui::Button("Publish Cones", {-1, 0})) {
                    sim->PublishCones(shuffle_cones, cones_to_send);
                }
                ImGui::EndDisabled();

                ImGui::TableSetColumnIndex(1);
                if (ImGui::Button("Update Path Planning", {-1, 0}))
                    sim->UpdatePathPlanning();

                ImGui::EndTable();
            }

            ImGui::SeparatorText("Simulation Control");
            if (sim->path_planning_center_points.empty())
                sim->auto_update_car_pos = false;
            if (ImGui::BeginTable("##PathPlanningSim", 3,
                                  ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableNextRow();

                ImGui::BeginDisabled(sim->persistent_path_planning_center_points.empty());

                ImGui::TableSetColumnIndex(0);
                if (ImGui::ArrowButton("Step Sim Backward", ImGuiDir_Left))
                    sim->NextPosition_PathPlanning(-1);

                ImGui::EndDisabled();

                ImGui::BeginDisabled(sim->path_planning_center_points.empty());

                ImGui::TableSetColumnIndex(1);
                if (ImGui::Button(sim->auto_update_car_pos ? "Stop" : "Play", {-1, 0})) {
                    sim->auto_update_car_pos = !sim->auto_update_car_pos;
                }

                ImGui::TableSetColumnIndex(2);
                if (ImGui::ArrowButton("Step Sim Forward", ImGuiDir_Right))
                    sim->NextPosition_PathPlanning(1);

                ImGui::EndDisabled();

                ImGui::EndTable();
            }
            // if (sim->path_planning_center_points.empty())
            //     sim->auto_update_car_pos = false;
            // if (ImGui::Button("Step Path Planning Forwards") || sim->auto_update_car_pos) {
            //     sim->NextPosition_PathPlanning();
            // }
            // ImGui::Checkbox("Auto Update", &sim->auto_update_car_pos);
            // if (ImGui::Button("Step Path Planning Backwards")) {
            //     sim->NextPosition_PathPlanning(-1);
            // }

            ImGui::SeparatorText("Import/Export");

            if (ImGui::BeginTable("##PathPlanningUpdateButtons", 2,
                                  ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Button("Export Cones", {-1, 0})) {
                    ConfigHelper::SaveConesCSV("", *sim);
                }

                ImGui::TableSetColumnIndex(1);
                if (ImGui::Button("Export Track", {-1, 0})) {
                    ConfigHelper::SaveTrackCSV("", *sim);
                }

                ImGui::EndTable();
            }

            if (ImGui::Button("Import Track", {-1, 0})) {
                ConfigHelper::LoadTrackCSV("", *sim);
                sim->Recalculate_Bezier();
                sim->Recalculate_Cones();
            }

            // ImGui::SeparatorText("Debug");
            //
            // if (ImGui::Button("Optimise parameters", {-1, 0})) {
            //     sim->PublishTrackTrueCenter();
            // }
        }
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // -------------------------- RIGHT SIDE CANVAS --------------------------
    window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar;
    ImGui::SetNextWindowSizeConstraints({200, 200}, {4000, 4000});
    ImGui::SetNextWindowSize({-1, all_avail.y});
    if (ImGui::BeginChild("RightCanvas", {-1, -1}, ImGuiChildFlags_None, window_flags)) {
        Vec2 avail = ImGui::GetContentRegionAvail() + style.WindowPadding * 2;;
        Vec2 pos = ImGui::GetWindowPos();
        auto dl = ImGui::GetWindowDrawList();
        static Vec2 *dragged_point{};
        ImGui::HandleCanvasActions(canvas_offset, canvas_zoom,
                                   adding_points ? ImGuiMouseButton_Right : ImGuiMouseButton_Left);
        ImGui::DrawCanvas(pos, avail, canvas_offset, canvas_zoom);

        if (!ImGui::IsMouseDown(0) && dragged_point) {
            if (dragged_point == &sim->car_position)
                dragged_point = nullptr;
            else {
                if (sim->smooth_curve && sim->center_points.size() > 1)
                    sim->Recalculate_Bezier();
                sim->Recalculate_Cones();
                dragged_point = nullptr;
            }
        }

        // Draw Track points
        if (show_track_points) {
            for (int i = 0; i < sim->center_points.size(); i++) {
                auto &p = sim->center_points[i];
                ImGui::PushID(i);
                ImGui::SetNextItemAllowOverlap();
                ImGui::DrawPoint(ImGui::World2Canvas(p, pos + canvas_offset, canvas_zoom), nullptr, 4,
                                 i == 0 ? CONE_COLOR_START_ORANGE : TRACK_COLOR_LINE_BASE);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                        dragged_point = &p;
                }
                if (ImGui::BeginPopupContextItem()) {
                    ImGui::Text("Point Settings");
                    if (ImGui::Button("Remove Point", {-1, 0})) {
                        sim->center_points.erase(sim->center_points.begin() + i);
                        if (sim->smooth_curve)
                            sim->Recalculate_Bezier();
                        else if (sim->center_points.size() > 1)
                            sim->Recalculate_Cones();
                        cones_to_send = std::min(cones_to_send, (int)std::max(sim->cones_left_positions.size(), sim->cones_right_positions.size()));
                        ImGui::CloseCurrentPopup();
                    }
                    if (ImGui::Checkbox("Manual Bezier", &sim->center_points[i].manual_setup))
                        if (!sim->center_points[i].manual_setup)
                            sim->Recalculate_Bezier();
                    ImGui::EndPopup();
                }

                if (sim->center_points[i].manual_setup && sim->smooth_curve) {
                    for (int j = 0; j < 2; j++) {
                        ImGui::PushID(j);
                        //auto& cp = sim->bezier_control_points[(i + sim->center_points.size() - j) % sim->center_points.size()][j];
                        auto &cp = sim->bezier_control_points[i][j];
                        ImGui::DrawPoint(ImGui::World2Canvas(cp, pos + canvas_offset, canvas_zoom),
                                         nullptr, 4, POINT_BEZIER_CONTROL_COLOR);

                        if (ImGui::IsItemHovered()) {
                            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                                dragged_point = &cp;
                        }

                        ImGui::PopID();
                    }
                }

                ImGui::PopID();
            }

            // Adding points
            if (adding_points && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && (
                    !ImGui::IsAnyItemHovered() && (!ImGui::IsAnyItemActive() || ImGui::GetActiveID() ==
                                                   ImGui::GetCurrentWindow()->MoveId) && ImGui::IsWindowHovered())) {
                Vec2 trans_pos = ImGui::Canvas2World(ImGui::GetMousePos(), pos + canvas_offset, canvas_zoom);
                //printf("added point at (%.2f, %.2f), mouse pos: (%.2f, %.2f)\n", trans_pos.x, trans_pos.y,
                //       ImGui::GetMousePos().x, ImGui::GetMousePos().y);

                // Check if the point is on the track first
                if (int best = sim->GetClosestPointOnTheTrack(trans_pos, ADD_ON_TRACK_DIST_THRESHOLD); best >= 0) {
                    sim->center_points.insert(sim->center_points.begin() + best, trans_pos);
                }
                else {
                    sim->center_points.emplace_back(trans_pos);
                }

                // Update cones_to_send if it was set to max, update to a new max
                bool cones_to_send_max = cones_to_send == std::max(sim->cones_left_positions.size(), sim->cones_right_positions.size());

                if (sim->smooth_curve && sim->center_points.size() > 1)
                    sim->Recalculate_Bezier();
                else if (sim->center_points.size() > 1)
                    sim->Recalculate_Cones();

                if (cones_to_send_max)
                    cones_to_send = std::max(sim->cones_left_positions.size(), sim->cones_right_positions.size());
            }
        }

        // Draw the Track itself
        if (show_track_center_line) {
            if (sim->center_points.size() > 1) {
                auto n = sim->center_points.size();
                for (int i = 0; i < n - 1; i++) {
                    Vec2 p0 = ImGui::World2Canvas(sim->center_points[i], pos + canvas_offset, canvas_zoom),
                            p3 = ImGui::World2Canvas(sim->center_points[i + 1], pos + canvas_offset, canvas_zoom);
                    if (sim->smooth_curve) {
                        Vec2 p1 = ImGui::World2Canvas(sim->bezier_control_points[i][0], pos + canvas_offset, canvas_zoom),
                                p2 = ImGui::World2Canvas(sim->bezier_control_points[i][1], pos + canvas_offset, canvas_zoom);
                        dl->AddBezierCubic(p0, p1, p2, p3, TRACK_COLOR_LINE_BASE, LINE_BASE_THICKNESS);
                    } else {
                        dl->AddLine(p0, p3, TRACK_COLOR_LINE_BASE, LINE_BASE_THICKNESS);
                    }
                }
                if (sim->close_loop) {
                    Vec2 p0 = ImGui::World2Canvas(sim->center_points[n - 1], pos + canvas_offset, canvas_zoom),
                            p3 = ImGui::World2Canvas(sim->center_points[0], pos + canvas_offset, canvas_zoom);
                    if (sim->smooth_curve) {
                        Vec2 p1 = ImGui::World2Canvas(sim->bezier_control_points[n - 1][0], pos + canvas_offset, canvas_zoom),
                                p2 = ImGui::World2Canvas(sim->bezier_control_points[n - 1][1], pos + canvas_offset,
                                                         canvas_zoom);
                        dl->AddBezierCubic(p0, p1, p2, p3, TRACK_COLOR_LINE_BASE, LINE_BASE_THICKNESS);
                    } else {
                        dl->AddLine(p0, p3, TRACK_COLOR_LINE_BASE, LINE_BASE_THICKNESS);
                    }
                }
            }
        }
        //dl->PathStroke(TRACK_COLOR_LINE_BASE, 0, 1);


        // Draw Cones
        if (show_cones) {
            if (sim->center_points.size() > (sim->smooth_curve ? 2 : 1)) {
                //auto n = sim->cones_left_positions.size();
                for (int i = 0; i < sim->cones_left_positions.size(); i++) {
                    auto &cone = sim->cones_left_positions[i];
                    dl->AddCircleFilled(ImGui::World2Canvas(cone, pos + canvas_offset, canvas_zoom),
                                        CONE_SMALL_THICKNESS,
                                        (i != 0 && i <= 2) ? CONE_COLOR_START_ORANGE : CONE_COLOR_LEFT_BLUE);
                }
                for (int i = 0; i < sim->cones_right_positions.size(); i++) {
                    auto &cone = sim->cones_right_positions[i];
                    dl->AddCircleFilled(ImGui::World2Canvas(cone, pos + canvas_offset, canvas_zoom),
                                        CONE_SMALL_THICKNESS,
                                        (i != 0 &&i <= 2) ? CONE_COLOR_START_ORANGE : CONE_COLOR_RIGHT_YELLOW);
                }
            }
        }

        // Draw Path Planning Markers
        if (show_pp_markers) {
            for (int i = 0; i < sim->nice_path_planning_markers.size(); i++) {
                auto &m = sim->nice_path_planning_markers[i];
                ImGui::DrawPoint(ImGui::World2Canvas(m.pos, pos + canvas_offset, canvas_zoom), m._text.c_str(),
                                 4 * m.size, m._color, false);
            }
        }

        // Draw Global Path Planning path
        ImGui::SetNextItemAllowOverlap();
        if (show_pp_points) {
            ImGui::PushID("PP_Points");
            for (int i = 0; i < sim->persistent_path_planning_center_points.size(); i++) {
                ImGui::PushID(i);
                auto &p = sim->persistent_path_planning_center_points[i];
                ImGui::DrawPoint(ImGui::World2Canvas(p, pos + canvas_offset, canvas_zoom), nullptr, 4,
                                 PATH_PLANNING_GLOBAL_TRACK_POINT_COLOR);
                ImGui::SetItemTooltip("Path Planning point #%i", i);
                ImGui::PopID();
            }

            // Draw current Path Planning path
            for (int i = 0; i < sim->path_planning_center_points.size(); i++) {
                auto &p = sim->path_planning_center_points[i];
                ImGui::DrawPoint(ImGui::World2Canvas(p, pos + canvas_offset, canvas_zoom), nullptr, 4,
                                 PATH_PLANNING_TRACK_POINT_COLOR, false);
            }
            ImGui::PopID();
        }

        // Draw the Car
        if (show_car) {
            ImGui::DrawPoint(ImGui::World2Canvas(sim->car_position, pos + canvas_offset, canvas_zoom),
                             nullptr, 7, CAR_BASE_COLOR, true);
            Vec2 pointing = Vec2(sim->car_direction.getAxis().getX(), sim->car_direction.getAxis().getY()) * 100;
            pointing = sim->car_direction_euler * 2;
            //pointing.rotate(sim->car_direction.getAngle());
            ImGui::DrawArrow(dl, ImGui::World2Canvas(sim->car_position, pos + canvas_offset, canvas_zoom),
                             ImGui::World2Canvas(sim->car_position + pointing, pos + canvas_offset, canvas_zoom),
                             CAR_BASE_COLOR);
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
                dragged_point = &sim->car_position;
        }

        // Move the dragged point to the mouse
        if (dragged_point) {
            *dragged_point = ImGui::Canvas2World(ImGui::GetMousePos(), pos + canvas_offset, canvas_zoom);
        }

        // Draw mouse position on the screen
        if (ImGui::IsWindowHovered() || dragged_point) {
            Vec2 mouse_pos = ImGui::Canvas2World(ImGui::GetMousePos(), pos + canvas_offset, canvas_zoom);
            char buf[32];
            sprintf(buf, "(%.1f, %.1f)", mouse_pos.x, mouse_pos.y);
            ImGui::RenderTextClipped(pos - Vec2(0, 20) + Vec2(0, avail.y), pos + Vec2(150, 0) + Vec2(0, avail.y), buf,
                                     nullptr, nullptr, {0, 0.5});
        }
    }

    ImGui::EndChild();

    return 0;
}

void bolide_CoG_callback(const ros::TimerEvent &event) {
    if (sim->auto_update_car_pos)
        sim->NextPosition_PathPlanning(1);

    sim->UpdateCarPosition();
}

int main(int argc, char **argv) {
    GUI::Setup(OnGui);

    printf("2D Sim Started!\n");

    ImGui::GetIO().IniFilename = nullptr; // Disable config saving

    ros::init(argc, argv, "dv_2d_pp_sim_node");

    sim = new Simulation(12);

    std::thread rosThread([]() {
        ros::spin();
    });

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Timer timer = nh->createTimer(ros::Duration(0.1), bolide_CoG_callback);

    //printf("path: %s\n", (ros::package::getPath("dv_2d_pp_sim") + "/track_in.csv").c_str());

    // try to read the default track
    ConfigHelper::LoadTrackCSV(ros::package::getPath("dv_2d_pp_sim") + "/track_in.csv", *sim);
    // const char *homedir;
    // if ((homedir = getenv("HOME")) != nullptr) {
    //     ConfigHelper::LoadTrackCSV(std::string(homedir) + "/dv_ws/src/dv_2d_pp_sim/track_in.csv", *sim);
    // }
    if (!sim->center_points.empty()) {
        sim->Recalculate_Bezier();
        sim->Recalculate_Cones();
    }

    while (ros::ok()) {
        if (GUI::RenderFrame())
            break;
    }

    rosThread.detach(); // Provides a clean exit
    //rosThread.join();

    GUI::ShutDown();
}
