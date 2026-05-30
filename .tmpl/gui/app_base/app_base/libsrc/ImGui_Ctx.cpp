#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer.h"

#include "ImGui_Ctx.h"

/**
 * @brief ImGui 컨텍스트와 SDL2/SDLRenderer 백엔드를 초기화한다.
 *
 * @param[in] sdlCtx SDL 윈도우/렌더러를 포함한 컨텍스트
 * @return 성공 시 true, 실패 시 false
 */
bool InitImGui(SDL2Ctx &sdlCtx) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.LogFilename = nullptr;

  io.Fonts->AddFontFromFileTTF(
      "resource/fonts/NanumGothicCoding-Regular.ttf",
      18.0f,
      nullptr,
      io.Fonts->GetGlyphRangesKorean());

  if (!ImGui_ImplSDL2_InitForSDLRenderer(sdlCtx.pWindow, sdlCtx.pRenderer)) {
    return false;
  }
  if (!ImGui_ImplSDLRenderer_Init(sdlCtx.pRenderer)) {
    ImGui_ImplSDL2_Shutdown();
    return false;
  }
  return true;
}



/**
 * @brief ImGui 백엔드와 컨텍스트를 안전하게 종료한다.
 */
void CleanupImGui() {
  if (ImGui::GetCurrentContext() != nullptr) {
    ImGui_ImplSDLRenderer_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
  }
}


