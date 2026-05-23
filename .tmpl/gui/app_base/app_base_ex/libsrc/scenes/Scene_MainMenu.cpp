#include "Scene_MainMenu.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "imgui.h"

namespace Scene::MainMenu {

/**
 * @brief Main Menu 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::MainMenu);
}

/**
 * @brief Main Menu 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::MainMenu);
}

/**
 * @brief 매 프레임 Main Menu 씬 UI를 렌더링한다.
 *
 * "Start Scenario" → Scenarios 씬, "Option" → Option 씬으로 전이하며,
 * "Quit" 클릭 시 확인 팝업을 열고 "Yes" 선택 시 AppQuitRequest 를 enqueue 한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnRender(entt::registry &ECS, float dt) {
  (void)dt;
  RenderSceneChrome(ECS, SceneId::MainMenu, "Main Menu",
                   "간단한 버튼 UI와 종료 popup 예제를 제공한다.");

  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  if (ImGui::Button("Start Scenario", ImVec2(220.0f, 40.0f))) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::Scenarios});
  }
  if (ImGui::Button("Option", ImVec2(220.0f, 40.0f))) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::Option});
  }
  if (ImGui::Button("Quit", ImVec2(220.0f, 40.0f))) {
    ImGui::OpenPopup("Quit Confirmation");
  }

  if (ImGui::BeginPopupModal("Quit Confirmation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("애플리케이션을 종료하시겠습니까?");
    ImGui::Spacing();
    if (ImGui::Button("Yes", ImVec2(100.0f, 0.0f))) {
      dispatcher.enqueue<AppQuitRequest>(AppQuitRequest{});
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("No", ImVec2(100.0f, 0.0f))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  EndFullscreenUi();
}

}  // namespace Scene::MainMenu
