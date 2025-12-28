#pragma once

#include "../../include/External/imgui.h"
#include "../../include/common/Vec2.h"

#define LINE_BASE_COLOR ImGui::GetColorU32(ImGuiCol_Button)
#define LINE_SPECIAL_COLOR ImGui::GetColorU32(ImGuiCol_PlotHistogram)
#define LINE_INACTIVE_COLOR ImGui::GetColorU32(ImGuiCol_FrameBgHovered)
#define LINE_BASE_THICKNESS 2

#define CONE_COLOR_RIGHT_YELLOW (0xff2ee5d8)
#define CONE_COLOR_LEFT_BLUE (0xfff8ff2e)
#define CONE_COLOR_START_ORANGE (0xff05a9f0)

#define CONE_SMALL_THICKNESS 4
#define CONE_BIG_THICKNESS 8

#define CAR_BASE_COLOR (0xff020202)

#define TRACK_COLOR_LINE_BASE (0xF0404040)
#define PATH_PLANNING_TRACK_POINT_COLOR (0xffff00ff)
#define PATH_PLANNING_GLOBAL_TRACK_POINT_COLOR (0xffafa0af)

//#define POINT_BEZIER_CONTROL_COLOR (0xffff78a0)
#define POINT_BEZIER_CONTROL_COLOR (0xff2aad65)

#define POINT_BASE_COLOR LINE_BASE_COLOR
#define POINT_SPECIAL_COLOR LINE_SPECIAL_COLOR
#define POINT_BASE_RADIUS 8
#define POINT_CLOUD_BASE_RADIUS 2
//#define POINT_CLOUD_HULL_RADIUS 4

#define SHAPE_FILL_COLOR (LINE_BASE_COLOR - 0xCC000000)

#define CANVAS_CELL_SIZE 5.f

namespace tf2 {
    class Quaternion;
}

namespace ImGui {
    bool DrawPoint(Vec2 pos, const char* label, float radius = POINT_BASE_RADIUS, ImU32 col = POINT_BASE_COLOR, bool interactive = true);
    void DrawCanvas(Vec2 pos, Vec2 size, Vec2& offset, Vec2& zoom);
    void HandleCanvasActions(Vec2& canvas_offset, Vec2& canvas_zoom, ImGuiMouseButton interact_button);
    void DrawArrow(ImDrawList* dl, Vec2 start, Vec2 end, ImU32 col = LINE_BASE_COLOR, float thickness = 2);

    inline Vec2 World2Canvas(Vec2 pos, Vec2 offset, Vec2 zoom) {
        // Scale position by zoom, flip Y-axis, then apply offset and center
        Vec2 scaled = pos * zoom;
        scaled.y *= -1; // Flip Y-axis
        return scaled + offset;
    }

    inline Vec2 Canvas2World(Vec2 pos, Vec2 offset, Vec2 zoom) {
        // Adjust for offset and center, then invert scaling and Y-flip
        Vec2 adjusted = pos - offset;
        adjusted.y *= -1; // Flip Y-axis back
        return adjusted / zoom;
    }
};