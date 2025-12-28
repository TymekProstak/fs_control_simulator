#include "ImGuiExtensions.h"
#include "../../include/External/imgui_internal.h"
#include "../../include/common/Vec2.h"

bool ImGui::DrawPoint(Vec2 pos, const char *label, float radius, ImU32 col, bool interactive) {
    auto dl = GetWindowDrawList();

    if (label)
        RenderTextClipped(pos - Vec2(50, 20), pos + Vec2(50, 0), label, nullptr, nullptr, {0.5, 0});
    dl->AddCircleFilled(pos, radius, col);

    if (interactive) {
        SetCursorScreenPos(pos - Vec2(radius, radius));
        const bool button_res = InvisibleButton("P_Elem", ImVec2(2*radius, 2*radius));

        if (IsItemHovered())
            SetMouseCursor(ImGuiMouseCursor_Hand);

        return button_res;
    }

    // if(IsMouseHoveringRect(pos - Vec2(radius,radius), pos + Vec2(radius,radius))) {
    //     SetMouseCursor(ImGuiMouseCursor_Hand);
    //     return true;
    // }
    //
    // return false;

    return false;
}

void ImGui::DrawCanvas(Vec2 pos, Vec2 size, Vec2& offset, Vec2& zoom) {
    auto dl = GetWindowDrawList();

    // local space
    Vec2 offset_mod = Vec2(std::fmod(offset.x, CANVAS_CELL_SIZE*zoom.x), std::fmod(offset.y, CANVAS_CELL_SIZE*zoom.y));

    for(float x = 0; x - CANVAS_CELL_SIZE < size.x; x += CANVAS_CELL_SIZE * zoom.x) {
        Vec2 world = Canvas2World(Vec2(offset_mod.x + x, 0), offset, zoom);
        bool isCenterX = std::abs(world.x) < 0.01f;
        ImU32 lineColor = isCenterX ? GetColorU32(ImGuiCol_Separator) : GetColorU32(ImGuiCol_Border);

        dl->AddLine(pos + Vec2(offset_mod.x + x,0), pos + Vec2(offset_mod.x + x, size.y), GetColorU32(lineColor));
    }
    for (float y = 0; y - CANVAS_CELL_SIZE < size.y; y += CANVAS_CELL_SIZE * zoom.y) {
        Vec2 world = Canvas2World(Vec2(0, offset_mod.y + y), offset, zoom);
        bool isCenterY = std::abs(world.y) < 0.01f;
        ImU32 lineColor = isCenterY ? GetColorU32(ImGuiCol_Separator) : GetColorU32(ImGuiCol_Border);

        dl->AddLine(pos + Vec2(0, offset_mod.y + y), pos + Vec2(size.x, offset_mod.y + y), lineColor);
    }
}

void ImGui::HandleCanvasActions(Vec2& canvas_offset, Vec2& canvas_zoom, ImGuiMouseButton interact_button) {
    static bool is_dragging_canvas = false;
    if((!IsAnyItemHovered() && (!IsAnyItemActive() || GetActiveID() == GetCurrentWindow()->MoveId) && IsWindowHovered()) || is_dragging_canvas) {
        if(IsMouseDragging(interact_button)) {
            canvas_offset += GetMouseDragDelta(interact_button);
            ResetMouseDragDelta(interact_button);
            SetMouseCursor(ImGuiMouseCursor_Hand);
        }
        if(IsMouseDragging(interact_button, 0))
            is_dragging_canvas = true;

        // Either 1 or -1 (unless you can scroll fast enough)
        if (GetIO().MouseWheel != 0) {
            Vec2 mouse_pos_local = Canvas2World(GetMousePos(),
                                                GetWindowPos() + canvas_offset,
                                                canvas_zoom);

            // Apply zoom
            float zoom_factor = (GetIO().MouseWheel > 0) ? 1.5f : 0.75f;
            canvas_zoom *= zoom_factor;
            canvas_zoom.x = ImClamp(canvas_zoom.x, 1.f, 160.0f);
            canvas_zoom.y = ImClamp(canvas_zoom.y, 1.f, 160.0f);

            // Convert mouse position back to canvas space after zooming
            Vec2 new_mouse_canvas_pos = World2Canvas(mouse_pos_local,
                                                     GetWindowPos() + canvas_offset,
                                                     canvas_zoom);

            // Adjust offset so mouse position stays fixed
            canvas_offset += (GetMousePos() - new_mouse_canvas_pos);
        }

    }

    if(IsMouseReleased(interact_button))
        is_dragging_canvas = false;
}


void ImGui::DrawArrow(ImDrawList *dl, Vec2 start, Vec2 end, ImU32 col, float thickness) {
    dl->AddLine(start, end, col, thickness);

    constexpr float sin_45 = 0.707106781186547;
    Vec2 dir = Vec2(end - start);
    Vec2 rotated_dir1 = (Vec2(dir.x - dir.y, dir.x + dir.y) * sin_45).normal() * 10;
    dl->AddLine(end, end - rotated_dir1, col, thickness);

    Vec2 rotated_dir2 = (Vec2(dir.x + dir.y, -dir.x + dir.y) * sin_45).normal() * 10;
    dl->AddLine(end, end - rotated_dir2, col, thickness);
}