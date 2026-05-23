#include "Scene_Scenarios.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "imgui.h"
#include <string>

namespace Scene::Scenarios {

/**
 * @brief Scenarios 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::Scenarios);
}

/**
 * @brief Scenarios 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::Scenarios);
}

/**
 * @brief 매 프레임 Scenarios 씬 UI를 렌더링한다.
 *
 * "Scenario 1" ~ "Scenario 4" 버튼 클릭 시 선택값을 PlayArgs 에 담아
 * Play 씬으로의 전이를 요청한다. "Back To Main Menu" 로 복귀 가능하다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnRender(entt::registry &ECS, float dt) {
  (void)dt;
  RenderSceneChrome(ECS, SceneId::Scenarios, "Scenarios",
                   "Scenario 1~4 선택값은 ctx에 기록하고 Play 씬에서 텍스트로 보여준다.");

  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  for (int scenario = 1; scenario <= 4; ++scenario) {
    const std::string label = "Scenario " + std::to_string(scenario);
    if (ImGui::Button(label.c_str(), ImVec2(220.0f, 40.0f))) {
      dispatcher.enqueue<SceneTransitionRequest>(
          SceneTransitionRequest{SceneId::Play, PlayArgs{scenario}});
    }
  }
  if (ImGui::Button("Back To Main Menu", ImVec2(220.0f, 40.0f))) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::MainMenu});
  }

  EndFullscreenUi();
}

}  // namespace Scene::Scenarios
