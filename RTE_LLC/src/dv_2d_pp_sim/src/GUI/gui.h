#pragma once
#include "../../include/External/imgui.h"
#include "../../include/External/imgui_impl_glfw.h"
#include "../../include/External/imgui_impl_opengl3.h"

namespace GUI
{
	int Setup(int (*OnGui)());
    int RenderFrame();
    void ShutDown();

    void SetWindowSize(int x, int y);

    inline GLFWwindow* window;
}
