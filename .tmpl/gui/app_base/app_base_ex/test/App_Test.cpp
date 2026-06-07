#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <thread>

#include "CApp.h"
#include "SceneDef.h"
#include "Input_Imitation.h"

// ──────────────────────────────────────────────
// 헬퍼: 최대 timeout_ms 동안 조건이 참이 될 때까지 대기한다.
// ──────────────────────────────────────────────
static bool WaitFor(std::function<bool()> cond,
                    int timeout_ms = 3000,
                    int poll_ms    = 50) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cond()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
  }
  return cond();
}

// TEST_VISUAL=1 이면 타임아웃을 길게 잡는다 (커서 이동 + 딜레이 포함)
static int DefaultTimeout() {
  const char *v = std::getenv("TEST_VISUAL");
  return (v && v[0] == '1') ? 10000 : 3000;
}

// ──────────────────────────────────────────────
// 테스트 픽스처: 앱을 Start 하고 Title 씬 진입까지 대기
// ──────────────────────────────────────────────
class AppTest : public testing::Test {
protected:
  CApp app;
  std::shared_ptr<entt::registry> pECS;

  void SetUp() override {
    app.Start();
    pECS = app.GetECS();

    bool ok = WaitFor([this] {
      if (!pECS) return false;
      if (!pECS->ctx().contains<SceneRuntime>()) return false;
      const auto &rt = pECS->ctx().get<SceneRuntime>();
      return rt.activeScene == SceneId::Title &&
             rt.phase == TransitionPhase::None;
    }, DefaultTimeout());
    ASSERT_TRUE(ok) << "Title 씬 진입 타임아웃";
  }

  void TearDown() override {
    app.Stop();
    app.Wait();
  }

  const SceneRuntime &RT() const {
    return pECS->ctx().get<SceneRuntime>();
  }

  // 위젯이 WidgetRegistry 에 등록될 때까지 대기
  bool WaitWidget(const std::string &funcName, const std::string &itemName,
                  int timeout_ms = -1) {
    if (timeout_ms < 0) timeout_ms = DefaultTimeout();
    const std::string key = funcName + ":" + itemName;
    return WaitFor([this, &key] {
      if (!pECS->ctx().contains<WidgetRegistry_T>()) return false;
      return pECS->ctx().get<WidgetRegistry_T>().map.count(key) > 0;
    }, timeout_ms);
  }

  // 씬 전이 완료(phase == None)까지 대기
  bool WaitTransition(SceneId target, int timeout_ms = -1) {
    if (timeout_ms < 0) timeout_ms = DefaultTimeout();
    return WaitFor([this, target] {
      return RT().activeScene == target &&
             RT().phase == TransitionPhase::None;
    }, timeout_ms);
  }
};

// ──────────────────────────────────────────────
// TC-01: 앱 기동 시 Title 씬이 활성화되어 있어야 한다
// ──────────────────────────────────────────────
TEST_F(AppTest, TC01_InitialScene_IsTitle) {
  EXPECT_EQ(RT().activeScene, SceneId::Title);
  EXPECT_EQ(RT().phase, TransitionPhase::None);
}

// ──────────────────────────────────────────────
// TC-02: Title "Start to Main Menu" 클릭 → MainMenu 씬 전이
// ──────────────────────────────────────────────
TEST_F(AppTest, TC02_Title_StartButton_GoesToMainMenu) {
  ASSERT_TRUE(WaitWidget("UI_Title", "Start to Main Menu"))
      << "UI_Title:Start to Main Menu 위젯 등록 타임아웃";

  bool ok = Click_Evt(pECS, "UI_Title", "Start to Main Menu", SDL_BUTTON_LEFT);
  ASSERT_TRUE(ok) << "UI_Title:Start to Main Menu 위젯을 찾지 못함";

  ASSERT_TRUE(WaitTransition(SceneId::MainMenu))
      << "MainMenu 씬 전이 타임아웃";

  EXPECT_EQ(RT().activeScene, SceneId::MainMenu);
  EXPECT_EQ(RT().prevScene, SceneId::Title);
}

// ──────────────────────────────────────────────
// TC-03: MainMenu "Start Scenario" 클릭 → Scenarios 씬 전이
// ──────────────────────────────────────────────
TEST_F(AppTest, TC03_MainMenu_StartScenario_GoesToScenarios) {
  ASSERT_TRUE(WaitWidget("UI_Title", "Start to Main Menu"));
  Click_Evt(pECS, "UI_Title", "Start to Main Menu", SDL_BUTTON_LEFT);
  ASSERT_TRUE(WaitTransition(SceneId::MainMenu));

  ASSERT_TRUE(WaitWidget("UI_MainMenu", "Start Scenario"))
      << "UI_MainMenu:Start Scenario 위젯 등록 타임아웃";

  Click_Evt(pECS, "UI_MainMenu", "Start Scenario", SDL_BUTTON_LEFT);

  ASSERT_TRUE(WaitTransition(SceneId::Scenarios))
      << "Scenarios 씬 전이 타임아웃";

  EXPECT_EQ(RT().activeScene, SceneId::Scenarios);
  EXPECT_EQ(RT().prevScene,   SceneId::MainMenu);
}

// ──────────────────────────────────────────────
// TC-04: Scenarios "Back To Main Menu" 클릭 → MainMenu 복귀
// ──────────────────────────────────────────────
TEST_F(AppTest, TC04_Scenarios_Back_GoesToMainMenu) {
  // Title → MainMenu → Scenarios
  ASSERT_TRUE(WaitWidget("UI_Title", "Start to Main Menu"));
  Click_Evt(pECS, "UI_Title", "Start to Main Menu", SDL_BUTTON_LEFT);
  ASSERT_TRUE(WaitTransition(SceneId::MainMenu));

  ASSERT_TRUE(WaitWidget("UI_MainMenu", "Start Scenario"));
  Click_Evt(pECS, "UI_MainMenu", "Start Scenario", SDL_BUTTON_LEFT);
  ASSERT_TRUE(WaitTransition(SceneId::Scenarios));

  ASSERT_TRUE(WaitWidget("UI_ScenarioSelect", "Back To Main Menu"))
      << "UI_ScenarioSelect:Back To Main Menu 위젯 등록 타임아웃";

  Click_Evt(pECS, "UI_ScenarioSelect", "Back To Main Menu", SDL_BUTTON_LEFT);

  ASSERT_TRUE(WaitTransition(SceneId::MainMenu))
      << "MainMenu 복귀 전이 타임아웃";

  EXPECT_EQ(RT().activeScene, SceneId::MainMenu);
  EXPECT_EQ(RT().prevScene,   SceneId::Scenarios);
}

// ──────────────────────────────────────────────
// TC-05: Title 복귀 후 enterCounts[Title] >= 1
//
// 주의: Title 은 앱 최초 진입 씬이므로 InvokeOnEnter() 직접 호출로 시작한다.
//       enterCounts 는 씬 전이 Loading 완료 시에만 갱신된다.
//       따라서 Title → MainMenu → Title 복귀 후 카운트를 확인한다.
//       (app_base_ex 의 MainMenu 에는 Title 복귀 버튼이 없으므로
//        dispatcher 에 직접 SceneTransitionRequest 를 enqueue 한다.)
// ──────────────────────────────────────────────
TEST_F(AppTest, TC05_Title_EnterCount_AfterReturn) {
  // Title → MainMenu
  ASSERT_TRUE(WaitWidget("UI_Title", "Start to Main Menu"));
  Click_Evt(pECS, "UI_Title", "Start to Main Menu", SDL_BUTTON_LEFT);
  ASSERT_TRUE(WaitTransition(SceneId::MainMenu));

  // MainMenu → Title (dispatcher 직접 주입)
  ASSERT_TRUE(WaitWidget("UI_MainMenu", "Start Scenario"));
  {
    auto &dispatcher = pECS->ctx().get<entt::dispatcher>();
    dispatcher.enqueue<SceneTransitionRequest>(
        SceneTransitionRequest{SceneId::Title});
  }
  ASSERT_TRUE(WaitTransition(SceneId::Title))
      << "Title 복귀 전이 타임아웃";

  const auto &ec = RT().enterCounts;
  auto it = ec.find(SceneId::Title);
  ASSERT_NE(it, ec.end()) << "enterCounts 에 Title 키 없음";
  EXPECT_GE(it->second, 1);
}
