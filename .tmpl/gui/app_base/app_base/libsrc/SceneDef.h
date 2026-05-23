#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>

#include <SDL2/SDL_keycode.h>

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
 * @brief 씬 전이 중 로딩/정리 진행도를 씬과 App.cpp 간에 공유하는 컨텍스트이다.
 *
 * worker thread(OnEnter/OnExit)가 쓰고, main thread(렌더링)가 읽는다.
 * fProgress, szStatus 는 atomic 으로 thread-safe 하게 공유한다.
 *
 * szPhase/szCurrentScene/szTargetScene 은 main thread 전용 스냅샷이다.
 * 메인 스레드가 전이 시작 시점에 한 번 세팅하고 만다는 의미에서의 스냅샷
 * 전이 시작 시 App.cpp 가 설정하고 RenderTransitionScreen 이 읽는다.
 *
 * 사용 예:
 *   auto &lc = ECS.ctx().get<SceneLoadingContext>();
 *   lc.fProgress.store(0.5f);
 *   lc.szStatus.store("텍스처 로딩 중...");
 *
 * 주의: szStatus 에는 문자열 리터럴("...") 만 store 한다.
 *       로컬 변수나 std::string::c_str() 은 수명이 끝나면 dangling pointer 가 된다.
 */
struct SceneLoadingContext {
  std::atomic<float>        fProgress{0.0f};  ///< 진행도 (0.0 ~ 1.0)
  std::atomic<const char *> szStatus{""};     ///< 현재 작업 설명 (문자열 리터럴 전용, worker thread 쓰기)

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
  bool exitAfterFinishing = false;                ///< 정리 완료 후 앱 종료 여부
  bool shouldQuit = false;                        ///< 메인 루프 종료 여부 플래그
  std::optional<SceneId> prevScene;               ///< 직전 씬 (전이 시 자동 기록)
  std::optional<PlayArgs> playArgs;               ///< 다음 씬에 전달할 Play 씬 입력 데이터
  std::string lifecycleNote = "Application booted"; ///< 라이프사이클 디버그용 메모
  std::map<SceneId, int> enterCounts;             ///< 각 씬별 진입(Enter) 횟수
  std::map<SceneId, int> exitCounts;              ///< 각 씬별 이탈(Exit) 횟수
  std::thread workerThread;                       ///< OnEnter/OnExit worker thread (App.cpp 전용)
};

/**
 * @brief 앱 전역 누적 게임 상태. 씬 간에 유지되는 데이터를 담는다.
 *
 * 이 구조체는 게임 시작~종료까지 계속 누적된다.
 * entt ctx에 등록해 어느 씬에서든 접근 가능하다.
 *
 * 사용 예:
 * - emplace:  ECS.ctx().emplace<GameState>();
 * - 읽기:     auto& gs = ECS.ctx().get<GameState>();
 * - 수정:     ECS.ctx().get<GameState>().gold += 100;
 */
struct GameState {
  std::string playerName  = "Player";  ///< 플레이어 이름
  int         level       = 1;         ///< 현재 레벨 (씬 진행도 기준)
  int         score       = 0;         ///< 게임 전체 누적 점수
  int         gold        = 0;         ///< 보유 골드 (재화 예시)
  float       playTimeSec = 0.0f;      ///< 총 플레이 시간(초)
};

/**
 * @brief 매 프레임 갱신되는 입력 장치 상태이다.
 *
 * App.cpp 의 ProcessSdlEvents() 에서 SDL 이벤트 큐를 소진한 후 최신 상태로 유지한다.
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
