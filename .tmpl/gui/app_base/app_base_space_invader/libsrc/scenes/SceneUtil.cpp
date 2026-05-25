#include "SceneUtil.h"
#include "imgui.h"

void BeginFullscreenUi(const char *windowName) {
  ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(32.0f, 24.0f));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  ImGui::Begin(windowName, nullptr,
               ImGuiWindowFlags_NoDecoration |
               ImGuiWindowFlags_NoMove |
               ImGuiWindowFlags_NoResize |
               ImGuiWindowFlags_NoSavedSettings);
}

void EndFullscreenUi() {
  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(3);
}

void OnGenericSceneEnter(entt::registry &ECS, SceneId scene, float dt) {
  (void)dt;
  auto &runtime = ECS.ctx().get<SceneRuntime>();
  runtime.enterCounts[scene]++;
  runtime.lifecycleNote = std::string("Entered ") + ToString(scene);
}

void OnGenericSceneExit(entt::registry &ECS, SceneId scene, float dt) {
  (void)dt;
  auto &runtime = ECS.ctx().get<SceneRuntime>();
  runtime.exitCounts[scene]++;
  runtime.lifecycleNote = std::string("Exited ") + ToString(scene);
}
