#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <SDL2/SDL_keycode.h>
#include <SDL2/SDL_events.h>

#include "scenes/SceneMap.h"

/**
 * @brief 씬 전이 상태의 단계를 나타내는 열거형이다.
 */
enum class TransitionPhase {
  None,         ///< 전이 중이 아님
  TearingDown,  ///< 이전 씬 정리 단계
  Loading,    ///< 다음 씬 준비 단계
};

/**
 * @brief 전이 단계를 화면 표시용 문자열로 변환한다.
 */
inline const char *ToString(const TransitionPhase phase) {
  switch (phase) {
    case TransitionPhase::None:        return "None";
    case TransitionPhase::TearingDown: return "TearingDown";
    case TransitionPhase::Loading:   return "Loading";
  }
  return "Unknown";
}

/**
 * @brief Play 씬 진입 시 전달할 초기화 데이터이다.
 *
 * 새 씬에 전달할 데이터가 필요하면 동일한 패턴으로 XxxArgs 구조체를 추가하고
 * SceneTransitionRequest 와 SceneRuntime 에 optional<XxxArgs> 를 추가한다.
 */
struct PlayArgs {
  int selectedScenario = 0; ///< 선택된 시나리오 번호
};

/**
 * @brief 씬 전이를 요청할 때 사용하는 이벤트 페이로드 구조체이다.
 *
 * target 씬에 초기화 데이터가 필요한 경우 해당 XxxArgs 필드를 채워서 enqueue 한다.
 * 데이터가 필요 없는 전환은 target 만 지정하면 된다.
 */
struct SceneTransitionRequest {
  SceneId target;                   ///< 이동할 대상 씬
  std::optional<PlayArgs> playArgs; ///< Play 씬 진입 시 전달 데이터 (선택적)
};

/**
 * @brief 애플리케이션 종료를 요청할 때 사용하는 이벤트 페이로드 구조체이다.
 */
struct AppQuitRequest {
};

/**
 * @brief 씬 전이 중 로딩/정리 진행도를 씬 태스크와 CApp.cpp 간에 공유하는 컨텍스트이다.
 *
 * OnEnter/OnExit 태스크(메인 스레드)가 쓰고, RenderTransitionScreen(메인 스레드)이 읽는다.
 * 모두 메인 스레드에서 동작하므로 atomic 이 필요 없다.
 *
 * szPhase/szCurrentScene/szTargetScene 은 전이 시작 시 CApp.cpp 가 설정하는 스냅샷이다.
 *
 * 사용 예 (OnEnter/OnExit 태스크 내부):
 *   auto &lc = ECS.ctx().get<SceneLoadingContext>();
 *   lc.fProgress = 0.5f;
 *   lc.szStatus  = "텍스처 로딩 중...";
 *
 * 주의: szStatus 에는 문자열 리터럴("...") 만 대입한다.
 *       로컬 변수나 std::string::c_str() 은 수명이 끝나면 dangling pointer 가 된다.
 */
struct SceneLoadingContext {
  float        fProgress = 0.0f;  ///< 진행도 (0.0 ~ 1.0) — 메인 스레드 전용
  const char * szStatus  = "";    ///< 현재 작업 설명 (문자열 리터럴 전용)

  // 전이 화면 표시용 스냅샷 — main thread 전용 (non-atomic)
  const char *szPhase        = "";  ///< ToString(phase) 스냅샷
  const char *szCurrentScene = "";  ///< ToString(activeScene) 스냅샷
  const char *szTargetScene  = "";  ///< ToString(pendingScene) 또는 "Application Exit"
};

/**
 * @brief 현재 씬 런타임 상태와 전이 정보, 통계를 기록하는 구조체이다.
 *
 * EnTT ctx 에 등록되어 어느 씬에서든 ECS.ctx().get<SceneRuntime>() 으로 접근한다.
 */
struct SceneRuntime {
  SceneId activeScene = SceneId::Title;           ///< 현재 활성화된 씬
  std::optional<SceneId> pendingScene;            ///< 대기 중인 다음 씬
  TransitionPhase phase = TransitionPhase::None;  ///< 현재 씬 전이 단계
  bool phaseScreenPresented = false;              ///< 현재 phase 화면이 최소 1프레임 노출되었는지 여부
  bool phaseWorkDone = false;                     ///< 현재 phase의 핵심 작업(OnExit/OnEnter) 완료 여부
  bool phaseWorkStarted = false;                  ///< OnEnter/OnExit 호출 및 태스크 큐 채우기 완료 여부
  bool exitAfterFinishing = false;                ///< 정리 완료 후 앱 종료 여부
  bool shouldQuit = false;                        ///< 메인 루프 종료 여부 플래그
  std::optional<SceneId> prevScene;               ///< 직전 씬 (전이 시 자동 기록)
  std::optional<PlayArgs> playArgs;               ///< 다음 씬에 전달할 Play 씬 입력 데이터
  std::string lifecycleNote = "Application booted"; ///< 라이프사이클 디버그용 메모
  std::map<SceneId, int> enterCounts;             ///< 각 씬별 진입(Enter) 횟수
  std::map<SceneId, int> exitCounts;              ///< 각 씬별 이탈(Exit) 횟수
};



/**
 * @brief 매 프레임 갱신되는 입력 장치 상태이다.
 *
 * CApp.cpp 의 ProcessSdlEvents() 에서 SDL 이벤트 큐를 소진한 후 최신 상태로 유지한다.
 * 씬의 OnUpdate() 에서 ECS.ctx().get<InputState>() 로 읽기 전용 접근한다.
 *
 * 동시 입력 감지 예시:
 *   const auto &input = ECS.ctx().get<InputState>();
 *   if (input.IsKeyHeld(SDLK_w) && input.IsKeyHeld(SDLK_a)) { ... }
 */
struct InputState {
  std::unordered_set<SDL_Keycode> keysHeld;        ///< 현재 눌린 키 집합
  std::unordered_set<int>         joyButtonsHeld;  ///< 현재 눌린 조이스틱 버튼 집합
  float joyAxisX = 0.0f;  ///< 조이스틱 X 축 (-1.0 ~ 1.0)
  float joyAxisY = 0.0f;  ///< 조이스틱 Y 축 (-1.0 ~ 1.0)

  // 마우스 버튼 (SDL_BUTTON_LEFT=1, SDL_BUTTON_MIDDLE=2, SDL_BUTTON_RIGHT=3)
  std::unordered_set<int> mouseButtonsHeld;      ///< 현재 누르고 있는 버튼
  std::unordered_set<int> mouseButtonsPressed;   ///< 이번 프레임에 누른 버튼 (1회성, 매 프레임 clear)
  std::unordered_set<int> mouseButtonsReleased;  ///< 이번 프레임에 뗀 버튼 (1회성, 매 프레임 clear)

  // 마우스 위치 / 휠
  int mouseX      = 0;  ///< 현재 커서 X (픽셀)
  int mouseY      = 0;  ///< 현재 커서 Y (픽셀)
  int wheelDeltaY = 0;  ///< 이번 프레임 휠 스크롤 (+위/-아래, 매 프레임 clear)

  bool IsKeyHeld(SDL_Keycode key) const    { return keysHeld.count(key) > 0; }
  bool IsJoyButtonHeld(int btn) const      { return joyButtonsHeld.count(btn) > 0; }
  bool IsMouseHeld(int btn) const          { return mouseButtonsHeld.count(btn) > 0; }
  bool IsMousePressed(int btn) const       { return mouseButtonsPressed.count(btn) > 0; }
  bool IsMouseReleased(int btn) const      { return mouseButtonsReleased.count(btn) > 0; }
};

/**
 * @brief 위젯의 화면 좌표를 저장하는 항목이다.
 *
 * REG_WIDGET 매크로로 UI 함수 내에서 등록하고,
 * 테스트에서 위젯 이름으로 좌표를 조회해 가상 클릭을 수행한다.
 */
struct WidgetPos {
  int iX = 0;  ///< 화면 X 좌표 (픽셀)
  int iY = 0;  ///< 화면 Y 좌표 (픽셀)
};

/**
 * @brief 위젯 좌표 레지스트리이다.
 *
 * entt ctx 에 등록되며 "함수명:위젯명" 키로 WidgetPos 를 보관한다.
 * UI_XXX 함수 내에서 REG_WIDGET 매크로로 자동 등록된다.
 *
 * 사용 예:
 *   auto &reg = ECS.ctx().get<WidgetRegistry_T>();
 *   auto it   = reg.map.find("UI_Title:Click Here");
 */
struct WidgetRegistry_T {
  std::map<std::string, WidgetPos> map;  ///< "함수명:위젯명" -> 좌표
};

/**
 * @brief 테스트용 외부 SDL 이벤트 주입 컨텍스트이다.
 *
 * 테스트에서 vecEvents 에 SDL_Event 를 push 하면
 * ProcessSdlEvents() 가 매 프레임 1개씩 SDL_PushEvent 로 주입한다.
 */
struct ExtEvt_T {
  std::vector<SDL_Event> vecEvents;  ///< 주입 대기 중인 SDL 이벤트 목록
};

/**
 * @brief 현재 UI 함수 이름과 위젯 이름으로 WidgetRegistry_T 에 좌표를 등록한다.
 *
 * @param ECS   entt::registry (ctx 에 WidgetRegistry_T 가 emplace 되어 있어야 한다)
 * @param Name  위젯 식별 이름 (문자열 리터럴)
 * @param X     화면 X 좌표
 * @param Y     화면 Y 좌표
 */
#define REG_WIDGET(ECS, Name, X, Y)                                       \
  do {                                                                     \
    if ((ECS).ctx().contains<WidgetRegistry_T>()) {                       \
      std::string _key = std::string(__FUNCTION__) + ":" + (Name);        \
      (ECS).ctx().get<WidgetRegistry_T>().map[_key] = WidgetPos{(X),(Y)}; \
    }                                                                      \
  } while (0)
