#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "CApp.h"
#include "CBase.h"
#include "Log_Util.h"
#include "SDL2_Ctx.h"
#include "ImGui_Ctx.h"
#include "SceneDef.h"

#include "scenes/SceneUtil.h"
#include "scenes/SceneMap.h"

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer.h"

namespace {

/// @brief 메인 루프의 목표 루프/초 (frames per second).
constexpr double APP_LPS = 60.0;

/**
 * @brief RGB 색상 값을 담는 구조체이다.
 */
struct ColorRgb {
  Uint8 r; ///< Red 컴포넌트
  Uint8 g; ///< Green 컴포넌트
  Uint8 b; ///< Blue 컴포넌트
};

/// @brief 일반 씬 배경색 (중간 회색).
constexpr ColorRgb kSceneBackground{110, 110, 110};
/// @brief 씬 전이 중 배경색 (어두운 회색)
constexpr ColorRgb kTransitionBackground{60, 60, 60};

}  // namespace


class CApp::Impl {
public:
  /**
   * @brief ECS 컨텍스트, 디스패처, 씬 정의를 초기화하고 이벤트를 연결한다.
   */
  Impl()
      : m_pECS(std::make_shared<entt::registry>()) {
    m_pECS->ctx().emplace<SDL2Ctx>();
    m_pECS->ctx().emplace<entt::dispatcher>();
    m_pECS->ctx().emplace<SceneRuntime>();
    m_pECS->ctx().emplace<SceneLoadingContext>();
    m_pECS->ctx().emplace<InputState>();  // 매 프레임 갱신되는 입력 장치 상태
    m_pECS->ctx().emplace<WidgetRegistry_T>();  // 위젯 좌표 레지스트리 (테스트용)
    m_pECS->ctx().emplace<ExtEvt_T>();          // 외부 이벤트 주입 버퍼 (테스트용)

    SetupSceneMap();
    SetupProcedureMap();

    auto &dispatcher = m_pECS->ctx().get<entt::dispatcher>();
    dispatcher.sink<SceneTransitionRequest>().connect<&CApp::Impl::OnSceneTransitionRequest>(*this);
    dispatcher.sink<AppQuitRequest>().connect<&CApp::Impl::OnAppQuitRequest>(*this);
  }

  /**
   * @brief 등록했던 dispatcher sink 를 해제한다.
   */
  ~Impl() {
    if (m_pECS != nullptr && m_pECS->ctx().contains<entt::dispatcher>()) {
      auto &dispatcher = m_pECS->ctx().get<entt::dispatcher>();
      dispatcher.sink<SceneTransitionRequest>().disconnect<&CApp::Impl::OnSceneTransitionRequest>(*this);
      dispatcher.sink<AppQuitRequest>().disconnect<&CApp::Impl::OnAppQuitRequest>(*this);
    }
  }

  /**
   * @brief 메인 루프 시작 전에 SDL 과 ImGui 를 초기화한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @return 성공 시 0, 실패 시 1
   */
  int PreLoop(entt::registry &ECS) {
    auto &sdlCtx = ECS.ctx().get<SDL2Ctx>();
    if (Init_SDL_ctx(sdlCtx) != 0) {
      LER("SDL context initialization failed");
      ECS.ctx().get<SceneRuntime>().shouldQuit = true;
      Base.Set_bLoopTrigger(false);
      return 1;
    }

    if (!InitImGui(sdlCtx)) {
      LER("ImGui initialization failed");
      ECS.ctx().get<SceneRuntime>().shouldQuit = true;
      Base.Set_bLoopTrigger(false);
      return 1;
    }

    // SceneID::Title이 최초 진입점이다
    InvokeOnEnter(ECS, SceneId::Title);
    return 0;
  }

  /**
   * @brief 한 프레임의 입력 처리, 전이 갱신, 렌더링을 수행한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @return 종료 요청이 있으면 1, 계속 실행이면 0
   */
  int Loop(entt::registry &ECS) {
    auto &sdlCtx = ECS.ctx().get<SDL2Ctx>();
    auto &runtime = ECS.ctx().get<SceneRuntime>();

    double actualDiff_SEC = 1.0/APP_LPS;

    if (sdlCtx.pWindow == nullptr || sdlCtx.pRenderer == nullptr) {
      runtime.shouldQuit = true;
      return 1;
    }

    ProcessSdlEvents(ECS,m_LastDeltaTime_SEC);

    auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
    dispatcher.update<SceneTransitionRequest>();
    dispatcher.update<AppQuitRequest>();

    UpdateTransition(ECS,m_LastDeltaTime_SEC);
    UpdateActiveScene(ECS,m_LastDeltaTime_SEC);
    Render(ECS,m_LastDeltaTime_SEC);

    FPS_Ctrl(MAIN_APP, 1.0 / APP_LPS, actualDiff_SEC);
    m_LastDeltaTime_SEC = actualDiff_SEC;
    return runtime.shouldQuit ? 1 : 0;
  }

  /**
   * @brief 메인 루프 종료 후 ImGui 와 SDL 리소스를 정리한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @return 항상 0
   */
  int PostLoop(entt::registry &ECS) {
    CleanupImGui();
    auto &sdlCtx = ECS.ctx().get<SDL2Ctx>();
    Cleanup_SDL_ctx(sdlCtx);
    return 0;
  }

  /**
   * @brief 베이스 루프 스레드 종료를 대기한다.
   *
   * @return 대기 처리 결과 코드
   */
  int Wait() {
    return Base.Wait();
  }

  /**
   * @brief 현재 애플리케이션이 사용하는 루프 프로시저 맵을 반환한다.
   *
   * @return 프로시저 맵 참조
   */
  Proc_Map_T &ProcMap() {
    return m_procMap;
  }

  /**
   * @brief 씬 전이 요청 이벤트를 받아 실제 전이 절차를 시작한다.
   *
   * @param[in] request 전이 요청 정보
   */
  void OnSceneTransitionRequest(const SceneTransitionRequest &request) {
    if (request.playArgs.has_value()) {
      m_pECS->ctx().get<SceneRuntime>().playArgs = request.playArgs;
    }
    StartTransition(*m_pECS, request.target);
  }

  /**
   * @brief 종료 요청 이벤트를 받아 종료 전환 절차를 시작한다.
   */
  void OnAppQuitRequest(const AppQuitRequest &) {
    StartQuit(*m_pECS);
  }

  
  CBase Base;
  std::shared_ptr<entt::registry> m_pECS;

private:
  Proc_Map_T m_procMap;
  std::map<SceneId, SceneDefinition> m_sceneMap;
  double m_LastDeltaTime_SEC = 1.0/APP_LPS; // 측정된 마지막 time delta
                                            
  /**
   * @brief Pre/Loop/Post 루프 콜백 맵을 구성한다.
   */
  void SetupProcedureMap() {
    m_procMap = {
        {PRE_LOOP_PROC,  [this](entt::registry &ECS) { return PreLoop(ECS); }},
        {LOOP_PROC,      [this](entt::registry &ECS) { return Loop(ECS); }},
        {POST_LOOP_PROC, [this](entt::registry &ECS) { return PostLoop(ECS); }},
    };
  }

  /**
   * @brief 각 씬의 OnEnter/OnExit/Render 훅을 연결한다.
   */
  void SetupSceneMap() {
    m_sceneMap = GetSceneMap();
  }

  /**
   * @brief SDL 이벤트를 처리하고 필요한 경우 종료 이벤트를 enqueue 한다.
   *
   * 키보드/마우스/조이스틱 이벤트는 InputState ctx 에 즉시 반영한다.
   * SDL 이벤트 큐를 한 프레임에 전부 소진하므로 OnUpdate 호출 시점에는
   * 해당 프레임의 모든 입력이 반영되어 있다.
   *
   * pressed/released/wheelDeltaY 는 1회성 값이므로 이벤트 처리 전에 초기화한다.
   * ImGui 가 마우스/키보드를 소비 중인지는 씬에서 WantCaptureMouse/WantCaptureKeyboard 로 확인한다.
   *
   * @param[in,out] ECS            ECS 레지스트리
   * @param[in]     dbTimeDiff_SEC 실제 경과 시간(초)
   */
  void ProcessSdlEvents(entt::registry &ECS, double dbTimeDiff_SEC) {
    (void)dbTimeDiff_SEC;
    auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
    auto &input      = ECS.ctx().get<InputState>();

    // 외부 주입 이벤트: 테스트에서 ExtEvt_T.vecEvents 에 넣은 SDL_Event 를 1개씩 소진
    if (ECS.ctx().contains<ExtEvt_T>()) {
      auto &extEvt = ECS.ctx().get<ExtEvt_T>();
      if (!extEvt.vecEvents.empty()) {
        SDL_PushEvent(&extEvt.vecEvents.front());
        extEvt.vecEvents.erase(extEvt.vecEvents.begin());
      }
    }

    // 1회성 값 매 프레임 초기화
    input.mouseButtonsPressed.clear();
    input.mouseButtonsReleased.clear();
    input.wheelDeltaY = 0;

    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      ImGui_ImplSDL2_ProcessEvent(&event);

      switch (event.type) {
        case SDL_QUIT:
          dispatcher.enqueue<AppQuitRequest>(AppQuitRequest{});
          break;

        // 키보드
        case SDL_KEYDOWN:
          input.keysHeld.insert(event.key.keysym.sym);
          break;
        case SDL_KEYUP:
          input.keysHeld.erase(event.key.keysym.sym);
          break;

        // 마우스 버튼
        case SDL_MOUSEBUTTONDOWN:
          input.mouseButtonsHeld.insert(event.button.button);
          input.mouseButtonsPressed.insert(event.button.button);
          break;
        case SDL_MOUSEBUTTONUP:
          input.mouseButtonsHeld.erase(event.button.button);
          input.mouseButtonsReleased.insert(event.button.button);
          break;

        // 마우스 위치
        case SDL_MOUSEMOTION:
          input.mouseX = event.motion.x;
          input.mouseY = event.motion.y;
          break;

        // 마우스 휠
        case SDL_MOUSEWHEEL:
          input.wheelDeltaY += event.wheel.y;
          break;

        // 조이스틱
        case SDL_JOYBUTTONDOWN:
          input.joyButtonsHeld.insert(event.jbutton.button);
          break;
        case SDL_JOYBUTTONUP:
          input.joyButtonsHeld.erase(event.jbutton.button);
          break;
        case SDL_JOYAXISMOTION: {
          constexpr float kAxisScale = 1.0f / 32767.0f;
          if (event.jaxis.axis == 0)
            input.joyAxisX = event.jaxis.value * kAxisScale;
          else if (event.jaxis.axis == 1)
            input.joyAxisY = event.jaxis.value * kAxisScale;
          break;
        }
        default:
          break;
      }
    }
  }

  /**
   * @brief 씬 전이 타이머를 갱신하고 단계별 상태를 진행시킨다.
   *
   * TearingDown/Loading 페이즈에서 각각 exitTasks/enterTasks 큐를
   * 매 프레임 1개씩 메인 스레드에서 실행한다.
   * 태스크 실행 후 즉시 return 하여 다음 프레임에 로딩 화면이 렌더링되도록 한다.
   *
   * @param[in,out] ECS            ECS 레지스트리
   * @param[in]     dbTimeDiff_SEC 실제 경과 시간(초)
   */
  void UpdateTransition(entt::registry &ECS,
                        double dbTimeDiff_SEC) {
    (void)dbTimeDiff_SEC;
    auto &runtime = ECS.ctx().get<SceneRuntime>();
    if (runtime.phase == TransitionPhase::None) {
      return;
    }

    // 현재 phase 화면을 최소 1프레임 표시
    if (!runtime.phaseScreenPresented) {
      runtime.phaseScreenPresented = true;
      runtime.lifecycleNote = std::string("Transition screen presented for ") + ToString(runtime.phase);
      return;
    }

    if (runtime.phase == TransitionPhase::TearingDown) {
      if (!runtime.phaseWorkDone) {
        auto &lc = ECS.ctx().get<SceneLoadingContext>();

        // 최초 진입: OnExit 호출 → exitTasks 큐를 ECS.ctx() 에 등록
        if (!runtime.phaseWorkStarted) {
          runtime.phaseWorkStarted = true;
          lc.fProgress = 0.0f;
          lc.szStatus  = "정리 시작...";
          runtime.lifecycleNote = std::string("Running OnExit during ") + ToString(runtime.phase);
          InvokeOnExit(ECS, runtime.activeScene);
        }

        // 태스크 1개 실행
        if (ECS.ctx().contains<LoadTaskQueue>()) {
          auto &q = ECS.ctx().get<LoadTaskQueue>();
          if (!q.empty()) {
            q.front()(ECS);
            q.pop();
            return;
          }
          ECS.ctx().erase<LoadTaskQueue>();
        }

        // 큐 소진 → 완료
        runtime.phaseWorkDone = true;
        runtime.exitCounts[runtime.activeScene] += 1;
        if (runtime.exitAfterFinishing) {
          runtime.lifecycleNote = "Application exit teardown completed during TearingDown";
          runtime.shouldQuit = true;
        } else {
          runtime.lifecycleNote = std::string("OnExit completed during ") + ToString(runtime.phase);
          runtime.phase = TransitionPhase::Loading;
          runtime.phaseScreenPresented = false;
          runtime.phaseWorkDone = false;
          runtime.phaseWorkStarted = false;
          lc.szPhase = ToString(runtime.phase);
        }
        return;
      }
      return;
    }

    if (runtime.phase == TransitionPhase::Loading) {
      if (!runtime.pendingScene.has_value()) {
        return;
      }

      if (!runtime.phaseWorkDone) {
        auto &lc = ECS.ctx().get<SceneLoadingContext>();

        // 최초 진입: OnEnter 호출 → enterTasks 큐를 ECS.ctx() 에 등록
        if (!runtime.phaseWorkStarted) {
          runtime.phaseWorkStarted = true;
          lc.fProgress = 0.0f;
          lc.szStatus  = "로딩 시작...";
          runtime.lifecycleNote = std::string("Running OnEnter during ") + ToString(runtime.phase);
          InvokeOnEnter(ECS, *runtime.pendingScene);
        }

        // 태스크 1개 실행
        if (ECS.ctx().contains<LoadTaskQueue>()) {
          auto &q = ECS.ctx().get<LoadTaskQueue>();
          if (!q.empty()) {
            q.front()(ECS);
            q.pop();
            return;
          }
          ECS.ctx().erase<LoadTaskQueue>();
        }

        // 큐 소진 → 완료
        runtime.activeScene = *runtime.pendingScene;
        runtime.enterCounts[runtime.activeScene] += 1;
        runtime.pendingScene.reset();
        runtime.phase = TransitionPhase::None;
        runtime.phaseScreenPresented = false;
        runtime.phaseWorkDone = false;
        runtime.phaseWorkStarted = false;
        runtime.exitAfterFinishing = false;
        runtime.lifecycleNote = std::string("Showing ") + ToString(runtime.activeScene) + " scene";
        return;
      }
    }
  }

  /**
   * @brief 현재 프레임의 배경과 UI 를 렌더링한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     dbTimeDiff_SEC 실제 경과 시간(초)
   */
  void Render(entt::registry &ECS, double dbTimeDiff_SEC) {
    auto &sdlCtx = ECS.ctx().get<SDL2Ctx>();
    auto &runtime = ECS.ctx().get<SceneRuntime>();

    ImGui_ImplSDLRenderer_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const ColorRgb bg = (runtime.phase == TransitionPhase::None) ? 
                                       kSceneBackground : kTransitionBackground;
    SDL_SetRenderDrawColor(sdlCtx.pRenderer, bg.r, bg.g, bg.b, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(sdlCtx.pRenderer);

    if (runtime.phase == TransitionPhase::None) {
      RenderActiveScene(ECS, runtime,dbTimeDiff_SEC);
    } else {
      RenderTransitionScreen(ECS, runtime);
    }

    ImGui::Render();
    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
    SDL_RenderPresent(sdlCtx.pRenderer);
  }

  /**
   * @brief 현재 활성 씬의 렌더 함수를 호출한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     runtime 현재 씬 런타임 상태
   */
  void RenderActiveScene(entt::registry &ECS, SceneRuntime &runtime,
                         double dbTimeDiff_SEC) {
    auto sceneIter = m_sceneMap.find(runtime.activeScene);
    if (sceneIter != m_sceneMap.end()) {
      sceneIter->second.onRender(ECS, (float)dbTimeDiff_SEC);
    }
  }

  /**
   * @brief 전이 중이 아닐 때 활성 씬의 onUpdate 훅을 호출한다.
   *
   * onUpdate 가 nullptr 이면 조용히 스킵한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     dbTimeDiff_SEC 실제 경과 시간(초)
   */
  void UpdateActiveScene(entt::registry &ECS, double dbTimeDiff_SEC) {
    const auto &runtime = ECS.ctx().get<SceneRuntime>();
    if (runtime.phase != TransitionPhase::None) return;

    auto sceneIter = m_sceneMap.find(runtime.activeScene);
    if (sceneIter != m_sceneMap.end() && sceneIter->second.onUpdate) {
      sceneIter->second.onUpdate(ECS, (float)dbTimeDiff_SEC);
    }
  }

  /**
   * @brief 씬 전이 중 상태와 타겟 정보를 표시하는 화면을 렌더링한다.
   *
   * @param[in] runtime 현재 씬 전이 상태
   */
  void RenderTransitionScreen(entt::registry &ECS, const SceneRuntime &runtime) {
    auto &lc = ECS.ctx().get<SceneLoadingContext>();
    BeginFullscreenUi("TransitionScreen");
    ImGui::SetWindowFontScale(2.0f);
    ImGui::TextUnformatted(lc.szPhase);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Spacing();
    ImGui::Text("Current Scene: %s", lc.szCurrentScene);
    ImGui::Text("Target: %s",        lc.szTargetScene);
    ImGui::Spacing();
    ImGui::TextWrapped(
     "Transition order: TearingDown 화면 표시 -> current scene OnExit 수행"
     " -> Loading 화면 표시 -> target scene OnEnter 수행 -> target scene 화면");
    ImGui::Spacing();
    // 진행도 표시
    float progress      = lc.fProgress;
    const char *status  = lc.szStatus;
    ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f));
    ImGui::Spacing();
    if (status && status[0] != '\0') {
      ImGui::TextUnformatted(status);
    }
    ImGui::Spacing();
    ImGui::Text("Lifecycle: %s", runtime.lifecycleNote.c_str());
    EndFullscreenUi();
  }

  /**
   * @brief 현재 씬에서 대상 씬으로의 전이 절차를 시작한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     target 전이 대상 씬
   */
  void StartTransition(entt::registry &ECS, SceneId target) {
    auto &runtime = ECS.ctx().get<SceneRuntime>();
    if (runtime.shouldQuit || runtime.phase != TransitionPhase::None) {
      LWY("StartTransition: ignored, phase=" << ToString(runtime.phase)
          << " target=" << ToString(target));
      return;
    }
    if (runtime.activeScene == target) {
      return;
    }

    runtime.prevScene = runtime.activeScene;
    runtime.pendingScene = target;
    runtime.phase = TransitionPhase::TearingDown;
    runtime.phaseScreenPresented = false;
    runtime.phaseWorkDone = false;
    runtime.phaseWorkStarted = false;
    runtime.exitAfterFinishing = false;
    runtime.lifecycleNote = std::string("Starting ") + ToString(runtime.phase) + 
                            " for " + ToString(runtime.activeScene);

    // 전이 화면 스냅샷 설정 (main thread 전용)
    auto &lc = ECS.ctx().get<SceneLoadingContext>();
    lc.szPhase        = ToString(runtime.phase);
    lc.szCurrentScene = ToString(runtime.activeScene);
    lc.szTargetScene  = ToString(target);
  }

  /**
   * @brief 종료 요청에 대한 TearingDown 절차를 시작한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void StartQuit(entt::registry &ECS) {
    auto &runtime = ECS.ctx().get<SceneRuntime>();
    if (runtime.shouldQuit || runtime.phase != TransitionPhase::None) {
      return;
    }

    runtime.pendingScene.reset();
    runtime.phase = TransitionPhase::TearingDown;
    runtime.phaseScreenPresented = false;
    runtime.phaseWorkDone = false;
    runtime.phaseWorkStarted = false;
    runtime.exitAfterFinishing = true;
    runtime.lifecycleNote = std::string("Starting ") + ToString(runtime.phase) +
                            " for application exit";

    // 전이 화면 스냅샷 설정 (main thread 전용)
    auto &lc = ECS.ctx().get<SceneLoadingContext>();
    lc.szPhase        = ToString(runtime.phase);
    lc.szCurrentScene = ToString(runtime.activeScene);
    lc.szTargetScene  = "Application Exit";
  }

  /**
   * @brief 지정한 씬의 OnEnter 훅을 호출한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     scene 진입할 씬
   */
  void InvokeOnEnter(entt::registry &ECS, SceneId scene) {
    auto sceneIter = m_sceneMap.find(scene);
    if (sceneIter != m_sceneMap.end() && sceneIter->second.onEnter) {
      sceneIter->second.onEnter(ECS, 0.0f);
    }
  }

  /**
   * @brief 지정한 씬의 OnExit 훅을 호출한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     scene 이탈할 씬
   */
  void InvokeOnExit(entt::registry &ECS, SceneId scene) {
    auto sceneIter = m_sceneMap.find(scene);
    if (sceneIter != m_sceneMap.end() && sceneIter->second.onExit) {
      sceneIter->second.onExit(ECS, 0.0f);
    }
  }

};  // class CApp::Impl

/**
 * @brief 내부 구현 객체를 생성한다.
 */
CApp::CApp()
    : m_pImpl(std::make_unique<Impl>()) {
}

/**
 * @brief CApp 구현 객체를 기본 소멸 규칙으로 정리한다.
 */
CApp::~CApp() = default;

/**
 * @brief FPS 제어기를 초기화하고 메인 루프를 시작한다.
 *
 * @return 시작 결과 코드
 */
int CApp::Start() {
  Init_FPS_Ctrl();
  return m_pImpl->Base.Start(*m_pImpl->m_pECS, m_pImpl->ProcMap());
}

/**
 * @brief 메인 루프 중지를 요청한다.
 *
 * @return 중지 결과 코드
 */
int CApp::Stop() {
  return m_pImpl->Base.Stop();
}

/**
 * @brief 메인 루프 종료까지 대기한다.
 *
 * @return 대기 결과 코드
 */
int CApp::Wait() {
  return m_pImpl->Wait();
}

/**
 * @brief 내부 ECS 레지스트리를 반환한다.
 *
 * @return ECS 레지스트리 shared_ptr
 */
std::shared_ptr<entt::registry> CApp::GetECS() {
  return m_pImpl->m_pECS;
}
