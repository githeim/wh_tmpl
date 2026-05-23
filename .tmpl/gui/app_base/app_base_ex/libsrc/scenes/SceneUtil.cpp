#include "SceneUtil.h"

#include "imgui.h"

/**
 * @brief 화면 전체를 덮는 ImGui 창을 시작한다.
 *
 * 뷰포트 전체를 채우도록 위치/크기를 설정하고 장식을 모두 제거한 채
 * ImGui::Begin 을 호출한다. 반드시 EndFullscreenUi() 와 쌍으로 사용한다.
 *
 * @param[in] windowName ImGui 창 식별 이름
 */
void BeginFullscreenUi(const char *windowName) {
  ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(32.0f, 24.0f));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  ImGui::Begin(
      windowName,
      nullptr,
      ImGuiWindowFlags_NoDecoration |
          ImGuiWindowFlags_NoMove |
          ImGuiWindowFlags_NoResize |
          ImGuiWindowFlags_NoSavedSettings);
}

/**
 * @brief 전체 화면 ImGui 창을 종료하고 PushStyleVar/Color 스택을 복원한다.
 *
 * BeginFullscreenUi() 와 반드시 쌍으로 사용해야 한다.
 */
void EndFullscreenUi() {
  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(3);
}

/**
 * @brief 씬 공통 헤더(제목/설명/라이프사이클 정보)를 렌더링한다.
 *
 * BeginFullscreenUi() 를 호출하고 heading 을 대형 폰트로 표시한 뒤,
 * subtitle 과 씬별 OnEnter/OnExit 호출 횟수를 출력한다.
 * 이 함수는 ImGui 창을 열기만 하며 닫지 않는다 — 호출자가
 * 추가 위젯을 그린 뒤 EndFullscreenUi() 를 호출해야 한다.
 *
 * @param[in,out] ECS      ECS 레지스트리
 * @param[in]     scene    현재 씬 (카운터 조회에 사용)
 * @param[in]     heading  창 상단에 표시할 씬 제목 문자열
 * @param[in]     subtitle 씬 설명 문자열
 */
void RenderSceneChrome(entt::registry &ECS, SceneId scene,
                       const char *heading, const char *subtitle, float dt) {
  (void)dt;
  const auto &runtime = ECS.ctx().get<SceneRuntime>();
  BeginFullscreenUi(heading);
  ImGui::SetWindowFontScale(2.0f);
  ImGui::TextUnformatted(heading);
  ImGui::SetWindowFontScale(1.0f);
  ImGui::Spacing();
  ImGui::TextWrapped("%s", subtitle);
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Text("Lifecycle: %s", runtime.lifecycleNote.c_str());
  ImGui::Text("OnEnter count: %d",
              runtime.enterCounts.count(scene) ? runtime.enterCounts.at(scene) : 0);
  ImGui::Text("OnExit count: %d",
              runtime.exitCounts.count(scene) ? runtime.exitCounts.at(scene) : 0);
  ImGui::Spacing();
}

/**
 * @brief 공통 씬 진입 처리를 수행한다. (worker thread 에서 호출됨)
 *
 * SceneRuntime 을 건드리지 않는다. enterCounts/lifecycleNote 갱신은
 * App.cpp main thread 가 join 후에 수행한다.
 *
 * @param[in,out] ECS   ECS 레지스트리
 * @param[in]     scene 진입한 씬
 * @param[in] dt  프레임 경과 시간 (초, 미사용, 확장 예약)
 */
void OnGenericSceneEnter(entt::registry &ECS, SceneId scene, float dt) {
  (void)ECS;
  (void)scene;
  (void)dt;
}

/**
 * @brief 공통 씬 이탈 처리를 수행한다. (worker thread 에서 호출됨)
 *
 * SceneRuntime 을 건드리지 않는다. exitCounts/lifecycleNote 갱신은
 * App.cpp main thread 가 join 후에 수행한다.
 *
 * @param[in,out] ECS   ECS 레지스트리
 * @param[in]     scene 이탈한 씬
 * @param[in] dt  프레임 경과 시간 (초, 미사용, 확장 예약)
 */
void OnGenericSceneExit(entt::registry &ECS, SceneId scene, float dt) {
  (void)ECS;
  (void)scene;
  (void)dt;
}
