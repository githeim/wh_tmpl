#include "Scene_End.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "imgui.h"

namespace Scene::End {

/**
 * @brief End 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::End);
}

/**
 * @brief End 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::End);
}

/**
 * @brief 매 프레임 End 씬 UI를 렌더링한다.
 *
 * "To Main Menu" 버튼 클릭 시 Main Menu 씬으로의 전이를 요청한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnRender(entt::registry &ECS, float dt) {
  RenderSceneChrome(ECS, SceneId::End, "End",
                   "종료 화면 예제로 Main Menu 복귀 경로를 제공한다.");

  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  if (ImGui::Button("To Main Menu", ImVec2(220.0f, 40.0f))) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::MainMenu});
  }

  EndFullscreenUi();
}

}  // namespace Scene::End
