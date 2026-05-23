#include "Scene_Play.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "SDL2_Ctx.h"
#include "ECSUtil.h"

#include "imgui.h"

#include <SDL2/SDL.h>
#include <cmath>
#include "Log_Util.h"
namespace Scene::Play {

namespace {

// =============================================================================
// Components
// =============================================================================

/**
 * @brief 2D 위치와 회전 각도를 나타내는 컴포넌트이다.
 *
 * x, y 는 화면 비율 좌표 (0.0 ~ 1.0) 로 저장한다.
 * 실제 픽셀 좌표는 렌더 System 에서 윈도우 크기를 곱해 계산한다.
 */
struct Transform {
  float x     = 0.0f;  ///< 화면 X 비율 좌표 (0.0 ~ 1.0)
  float y     = 0.0f;  ///< 화면 Y 비율 좌표 (0.0 ~ 1.0, 게임 영역 기준)
  float angle = 0.0f;  ///< 회전 각도 (라디안)
};

/**
 * @brief 매 프레임 angle 에 더해지는 회전 속도를 나타내는 컴포넌트이다.
 */
struct RotationSpeed {
  float value = 0.0f;  ///< 라디안/초
};

/**
 * @brief 원 도형의 반지름을 나타내는 컴포넌트이다.
 *
 * PulseSystem 이 baseR 을 기준으로 매 프레임 r 을 갱신한다.
 */
struct CircleShape {
  float r     = 60.0f;  ///< 현재 반지름 (픽셀)
  float baseR = 60.0f;  ///< 맥박 기준 반지름
};

/**
 * @brief 등변 삼각형의 외접원 반지름을 나타내는 컴포넌트이다.
 */
struct TriangleShape {
  float r = 80.0f;  ///< 외접원 반지름 (픽셀)
};

/**
 * @brief 정사각형의 절반 크기를 나타내는 컴포넌트이다.
 */
struct RectShape {
  float half = 60.0f;  ///< 한 변의 절반 길이 (픽셀)
};

/**
 * @brief 씬 내 점수/틱 카운터를 나타내는 컴포넌트이다.
 */
struct ScoreCounter {
  int      score = 0;  ///< 씬 내 누적 점수
  uint64_t tick    = 0;    ///< OnUpdate 호출 횟수
  float    timeSec = 0.0f;  ///< 누적 경과 시간 (초)
};

/**
 * @brief SDL 드로잉 색상을 나타내는 컴포넌트이다.
 */
struct Color {
  uint8_t r = 255;  ///< Red
  uint8_t g = 255;  ///< Green
  uint8_t b = 255;  ///< Blue
};

/**
 * @brief Play 씬이 소유한 entity 임을 표시하는 태그이다.
 *
 * OnExit 에서 이 태그가 붙은 entity 만 일괄 destroy 한다.
 */
struct PlaySceneTag {};

// =============================================================================
// 씬별 커맨드 구조체 및 Apply
//
// 새 컴포넌트 추가 시:
//   1. CmdUpdateXxx 구조체 추가
//   2. Apply(reg, CmdUpdateXxx) 추가
//   3. PlayCmd variant 목록에 추가
// =============================================================================

/// Transform 값을 덮어쓰는 커맨드
struct CmdUpdateTransform   { entt::entity e; Transform    value; };
/// CircleShape 값을 덮어쓰는 커맨드
struct CmdUpdateCircleShape { entt::entity e; CircleShape  value; };
/// ScoreCounter 값을 덮어쓰는 커맨드
struct CmdUpdateScoreCounter{ entt::entity e; ScoreCounter value; };
/// RectShape angle 을 덮어쓰는 커맨드 (Transform 재사용)
struct CmdUpdateRectAngle   { entt::entity e; float angle; };

/// 기존 entity 에 RotationSpeed 컴포넌트를 추가하는 커맨드
struct CmdAddRotationSpeed    { entt::entity e; RotationSpeed value; };
/// 기존 entity 에서 RotationSpeed 컴포넌트를 제거하는 커맨드
struct CmdRemoveRotationSpeed { entt::entity e; };

inline void Apply(entt::registry &reg, const CmdUpdateTransform &c) {
  if (reg.valid(c.e) && reg.all_of<Transform>(c.e))
    reg.get<Transform>(c.e) = c.value;
}
inline void Apply(entt::registry &reg, const CmdUpdateCircleShape &c) {
  if (reg.valid(c.e) && reg.all_of<CircleShape>(c.e))
    reg.get<CircleShape>(c.e) = c.value;
}
inline void Apply(entt::registry &reg, const CmdUpdateScoreCounter &c) {
  if (reg.valid(c.e) && reg.all_of<ScoreCounter>(c.e))
    reg.get<ScoreCounter>(c.e) = c.value;
}
inline void Apply(entt::registry &reg, const CmdUpdateRectAngle &c) {
  if (reg.valid(c.e) && reg.all_of<Transform>(c.e))
    reg.get<Transform>(c.e).angle = c.angle;
}
inline void Apply(entt::registry &reg, const CmdAddRotationSpeed &c) {
  if (reg.valid(c.e)) reg.emplace_or_replace<RotationSpeed>(c.e, c.value);
}
inline void Apply(entt::registry &reg, const CmdRemoveRotationSpeed &c) {
  if (reg.valid(c.e) && reg.all_of<RotationSpeed>(c.e))
    reg.erase<RotationSpeed>(c.e);
}

/**
 * @brief Play 씬에서 사용하는 모든 커맨드 타입의 variant 이다.
 *
 * 공통 커맨드 (CmdDestroyEntity, CmdCreateEntity, CmdSubmit) 는 ECSUtil.h 에서,
 * 씬별 컴포넌트 커맨드는 이 목록에서 관리한다.
 */
using PlayCmd = std::variant<
  CmdDestroyEntity,
  CmdCreateEntity,
  CmdSubmit,
  CmdUpdateTransform,
  CmdUpdateCircleShape,
  CmdUpdateScoreCounter,
  CmdUpdateRectAngle,
  CmdAddRotationSpeed,
  CmdRemoveRotationSpeed
>;

/// Play 씬 전용 CommandBuffer 타입
using PlayCmdBuffer = CommandBuffer<PlayCmd>;

// =============================================================================
// SDL 도형 헬퍼
// =============================================================================

/**
 * @brief SDL Renderer 로 속이 찬 삼각형을 그린다. (scan-line 방식)
 *
 * 세 꼭짓점을 y 좌표 기준으로 정렬한 뒤 역기울기(inverse slope)로
 * 각 스캔라인의 좌우 끝점을 계산하여 수평선을 채운다.
 *
 * @param pRenderer SDL 렌더러
 * @param x1,y1     꼭짓점 1
 * @param x2,y2     꼭짓점 2
 * @param x3,y3     꼭짓점 3
 */
void DrawFilledTriangle(SDL_Renderer *pRenderer,
                        int x1, int y1,
                        int x2, int y2,
                        int x3, int y3) {
  auto sort2 = [](int &ax, int &ay, int &bx, int &by) {
    if (ay > by) { std::swap(ax, bx); std::swap(ay, by); }
  };
  sort2(x1, y1, x2, y2);
  sort2(x1, y1, x3, y3);
  sort2(x2, y2, x3, y3);

  auto scanLine = [&](int y, float lx, float rx) {
    int xl = static_cast<int>(lx), xr = static_cast<int>(rx);
    if (xl > xr) std::swap(xl, xr);
    SDL_RenderDrawLine(pRenderer, xl, y, xr, y);
  };

  float s1 = (y2 - y1) ? (float)(x2 - x1) / (y2 - y1) : 0.0f;
  float s2 = (y3 - y1) ? (float)(x3 - x1) / (y3 - y1) : 0.0f;
  float s3 = (y3 - y2) ? (float)(x3 - x2) / (y3 - y2) : 0.0f;

  for (int y = y1; y <= y2; ++y)
    scanLine(y, x1 + s1 * (y - y1), x1 + s2 * (y - y1));
  for (int y = y2; y <= y3; ++y)
    scanLine(y, x2 + s3 * (y - y2), x1 + s2 * (y - y1));
}

/**
 * @brief SDL Renderer 로 속이 찬 원을 그린다. (mid-point 방식)
 *
 * @param pRenderer SDL 렌더러
 * @param cx,cy     중심 좌표
 * @param r         반지름 (픽셀)
 */
void DrawFilledCircle(SDL_Renderer *pRenderer, int cx, int cy, int r) {
  for (int dy = -r; dy <= r; ++dy) {
    int dx = static_cast<int>(std::sqrt((float)(r * r - dy * dy)));
    SDL_RenderDrawLine(pRenderer, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

// =============================================================================
// Systems — Update (CommandBuffer 에 Add, 레지스트리 직접 수정 금지)
// =============================================================================

/**
 * @brief ScoreCounter.tick 을 증가시키고, 60틱마다 score +1 을 예약한다.
 *
 * 같은 프레임에서 PulseSystem 이 tick 을 읽을 때 아직 반영 전 값을 읽는 것은
 * 의도된 동작이다 — 같은 프레임 내 상태는 프레임 시작 스냅샷 기준으로 읽는다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (초, ScoreCounter.timeSec 누적에 사용)
 */
void ScoreSystem(entt::registry &ECS, float dt) {
  auto &cmb = ECS.ctx().get<PlayCmdBuffer>();
  for (auto [e, sc] : ECS.view<ScoreCounter>().each()) {
    ScoreCounter next = sc;
    next.tick++;
    next.timeSec += dt;
    if (next.tick % 60 == 0) next.score++;
    cmb.Add(CmdUpdateScoreCounter{e, next});
  }
}

/**
 * @brief Transform.angle 을 RotationSpeed.value * dt 만큼 증가시키는 커맨드를 예약한다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (초)
 */
void RotationSystem(entt::registry &ECS, float dt) {
  auto &cmb = ECS.ctx().get<PlayCmdBuffer>();
  for (auto [e, tf, rs] : ECS.view<Transform, RotationSpeed>().each()) {
    Transform next = tf;
    next.angle += rs.value * dt;
    if (next.angle > 6.2832f) next.angle -= 6.2832f;
    cmb.Add(CmdUpdateTransform{e, next});
  }
}

/**
 * @brief CircleShape.r 을 sin 맥박으로 갱신하는 커맨드를 예약한다.
 *
 * 현재 ScoreCounter.tick (프레임 시작 스냅샷) 을 기준으로 계산한다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (초)
 */
void PulseSystem(entt::registry &ECS, float dt) {
  (void)dt;
  auto &cmb = ECS.ctx().get<PlayCmdBuffer>();
  for (auto [e, cs, sc] : ECS.view<CircleShape, ScoreCounter>().each()) {
    CircleShape next = cs;
    next.r = cs.baseR + cs.baseR * 0.33f * std::sin(sc.timeSec * 3.0f);
    cmb.Add(CmdUpdateCircleShape{e, next});
  }
}

/**
 * @brief 전역 GameState 를 경과 시간과 씬 점수로 누적 갱신하는 커맨드를 예약한다.
 *
 * GameState 는 ctx 객체이므로 CmdSubmit 으로 처리한다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (초)
 */
void GlobalStateSystem(entt::registry &ECS, float dt) {
  auto &cmb      = ECS.ctx().get<PlayCmdBuffer>();
  const auto &gs = ECS.ctx().get<GameState>();

  // NOTE: ScoreSystem 은 tick++ 후 tick % 60 == 0 일 때 score +1 한다.
  // 여기서는 CommandBuffer 반영 전 스냅샷(sc.tick)을 읽으므로
  // "다음 tick 이 60의 배수" 조건인 (sc.tick + 1) % 60 == 0 으로 동기화한다.
  // ScoreSystem 의 점수 조건이 바뀌면 이 식도 함께 수정해야 한다.
  int scoreDelta = 0;
  for (auto [e, sc] : ECS.view<ScoreCounter>().each()) {
    scoreDelta += ((sc.tick + 1) % 60 == 0 ? 1 : 0);
  }

  float newTime  = gs.playTimeSec + dt;
  int   newScore = gs.score + scoreDelta;
  cmb.Add(CmdSubmit{[newTime, newScore](entt::registry &reg) {
    auto &g       = reg.ctx().get<GameState>();
    g.playTimeSec = newTime;
    g.score       = newScore;
  }});
}

/**
 * @brief InputState ctx 를 읽어 좌우 방향키로 사각형 entity 의 angle 을 갱신한다.
 *
 * - 좌방향키(SDLK_LEFT) : angle 감소 (반시계 회전)
 * - 우방향키(SDLK_RIGHT): angle 증가 (시계 회전)
 * - 좌우 동시 입력      : 서로 상쇄되어 각도 유지
 *
 * @param ECS 레지스트리 (읽기 전용, CommandBuffer 경유 수정)
 * @param dt  프레임 경과 시간 (초)
 */
void InputSystem(entt::registry &ECS, float dt) {
  const auto &input = ECS.ctx().get<InputState>();
  auto &cmb         = ECS.ctx().get<PlayCmdBuffer>();
  auto &dispatcher  = ECS.ctx().get<entt::dispatcher>();

  // END 키 — Game Over 전이
  if (input.IsKeyHeld(SDLK_END)) {
    LIG("Chk");
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::End});
  }

  constexpr float kRotSpeed = 2.0f;  // 라디안/초
  constexpr float kTwoPi    = 6.2832f;

  float delta = 0.0f;
  if (input.IsKeyHeld(SDLK_LEFT))  delta -= kRotSpeed * dt;
  if (input.IsKeyHeld(SDLK_RIGHT)) delta += kRotSpeed * dt;

  if (delta == 0.0f) return;

  for (auto [e, tf, rs] : ECS.view<Transform, RectShape>().each()) {
    float next = tf.angle + delta;
    if (next >  kTwoPi) next -= kTwoPi;
    if (next < -kTwoPi) next += kTwoPi;
    cmb.Add(CmdUpdateRectAngle{e, next});
  }
}


// =============================================================================
// Systems — Render (레지스트리 읽기 전용, CommandBuffer 사용 안 함)
// =============================================================================

/**
 * @brief TriangleShape + Transform + Color 를 읽어 SDL 로 삼각형을 그린다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (미사용)
 */
void RenderTriangleSystem(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx = ECS.ctx().get<AppCtx>();
  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);
  const int gameY = winH / 3;
  const int gameH = winH - gameY;

  auto view = ECS.view<Transform, TriangleShape, Color>();
  for (auto [e, tf, ts, col] : view.each()) {
    SDL_SetRenderDrawColor(appCtx.pRenderer, col.r, col.g, col.b, SDL_ALPHA_OPAQUE);
    int cx = static_cast<int>(tf.x * winW);
    int cy = gameY + static_cast<int>(tf.y * gameH);
    int r  = static_cast<int>(ts.r);
    DrawFilledTriangle(appCtx.pRenderer,
                       cx + static_cast<int>(r * std::sin(tf.angle)),
                       cy - static_cast<int>(r * std::cos(tf.angle)),
                       cx + static_cast<int>(r * std::sin(tf.angle + 2.094f)),
                       cy - static_cast<int>(r * std::cos(tf.angle + 2.094f)),
                       cx + static_cast<int>(r * std::sin(tf.angle + 4.189f)),
                       cy - static_cast<int>(r * std::cos(tf.angle + 4.189f)));
  }
}

/**
 * @brief CircleShape + Transform + Color 를 읽어 SDL 로 원을 그린다.
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (미사용)
 */
void RenderCircleSystem(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx = ECS.ctx().get<AppCtx>();
  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);
  const int gameY = winH / 3;
  const int gameH = winH - gameY;

  auto view = ECS.view<Transform, CircleShape, Color>();
  for (auto [e, tf, cs, col] : view.each()) {
    SDL_SetRenderDrawColor(appCtx.pRenderer, col.r, col.g, col.b, SDL_ALPHA_OPAQUE);
    DrawFilledCircle(appCtx.pRenderer,
                     static_cast<int>(tf.x * winW),
                     gameY + static_cast<int>(tf.y * gameH),
                     static_cast<int>(cs.r));
  }
}

/**
 * @brief RectShape + Transform + Color 를 읽어 회전된 사각형을 SDL 로 그린다.
 *
 * Transform.angle 을 기준으로 꼭짓점 4개를 계산하고,
 * 삼각형 2개(fan 분할)로 속이 찬 사각형을 렌더링한다.
 *
 * 꼭짓점 배치 (로컬 좌표, 반시계):
 *   v0(-h,-h)  v1(+h,-h)
 *   v3(-h,+h)  v2(+h,+h)
 * 삼각형 분할: [v0,v1,v2] + [v0,v2,v3]
 *
 * @param ECS 레지스트리 (읽기 전용)
 * @param dt  프레임 경과 시간 (미사용)
 */
void RenderRectSystem(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx = ECS.ctx().get<AppCtx>();
  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);
  const int gameY = winH / 3;
  const int gameH = winH - gameY;

  auto view = ECS.view<Transform, RectShape, Color>();
  for (auto [e, tf, rs, col] : view.each()) {
    SDL_SetRenderDrawColor(appCtx.pRenderer, col.r, col.g, col.b, SDL_ALPHA_OPAQUE);

    int cx = static_cast<int>(tf.x * winW);
    int cy = gameY + static_cast<int>(tf.y * gameH);
    float h  = rs.half;
    float ca = std::cos(tf.angle);
    float sa = std::sin(tf.angle);

    // 4개 꼭짓점 회전 변환
    // 로컬 (-h,-h),(+h,-h),(+h,+h),(-h,+h) → 월드 좌표
    auto rotX = [&](float lx, float ly) { return cx + static_cast<int>(lx * ca - ly * sa); };
    auto rotY = [&](float lx, float ly) { return cy + static_cast<int>(lx * sa + ly * ca); };

    int x0 = rotX(-h,-h), y0 = rotY(-h,-h);
    int x1 = rotX( h,-h), y1 = rotY( h,-h);
    int x2 = rotX( h, h), y2 = rotY( h, h);
    int x3 = rotX(-h, h), y3 = rotY(-h, h);

    // 삼각형 1: v0-v1-v2
    DrawFilledTriangle(appCtx.pRenderer, x0,y0, x1,y1, x2,y2);
    // 삼각형 2: v0-v2-v3
    DrawFilledTriangle(appCtx.pRenderer, x0,y0, x2,y2, x3,y3);
  }
}

/**
 * @brief ScoreCounter 컴포넌트와 GameState ctx 를 읽어 ImGui HUD 를 그린다.
 *
 * 상단 1/3 영역에 반투명 오버레이로 렌더링한다.
 * Game Over 버튼 클릭 시 SceneTransitionRequest 를 enqueue 한다.
 *
 * @param ECS 레지스트리 (읽기 전용, dispatcher enqueue 제외)
 * @param dt  프레임 경과 시간 (미사용)
 */
void HudSystem(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx        = ECS.ctx().get<AppCtx>();
  const auto &runtime = ECS.ctx().get<SceneRuntime>();
  const auto &gGs     = ECS.ctx().get<GameState>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);
  const int hudH = winH / 3;

  ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, static_cast<float>(hudH)));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(32.0f, 12.0f));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.55f));
  ImGui::Begin("PlayHUD", nullptr,
               ImGuiWindowFlags_NoDecoration |
               ImGuiWindowFlags_NoMove |
               ImGuiWindowFlags_NoResize |
               ImGuiWindowFlags_NoSavedSettings);

  ImGui::SetWindowFontScale(1.4f);
  ImGui::TextUnformatted("Play — HUD 영역 (상단 1/3)");
  ImGui::SetWindowFontScale(1.0f);
  ImGui::Separator();

  if (runtime.playArgs.has_value()) {
    ImGui::Text("Selected Scenario: %d", runtime.playArgs->selectedScenario);
  }

  auto view = ECS.view<ScoreCounter>();
  for (auto [e, sc] : view.each()) {
    ImGui::Text("Score: %d  Frame: %llu", sc.score, (unsigned long long)sc.tick);
  }

  ImGui::TextUnformatted("삼각형(R)회전  원(G)맥박  사각형(B)방향키 회전");
  ImGui::Text("Player: %s  Level: %d  Gold: %d  TotalTime: %.1fs",
              gGs.playerName.c_str(), gGs.level, gGs.gold, gGs.playTimeSec);

  const auto &input = ECS.ctx().get<InputState>();
  ImGui::Text("Mouse  X: %d  Y: %d  L:%s  M:%s  R:%s  Wheel:%d",
              input.mouseX, input.mouseY,
              input.IsMouseHeld(SDL_BUTTON_LEFT)   ? "ON" : "--",
              input.IsMouseHeld(SDL_BUTTON_MIDDLE) ? "ON" : "--",
              input.IsMouseHeld(SDL_BUTTON_RIGHT)  ? "ON" : "--",
              input.wheelDeltaY);
  ImGui::Spacing();

  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  if (ImGui::Button("Game Over", ImVec2(160.0f, 32.0f))) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::End});
  }

  // Add/Remove 컴포넌트 예시 — 삼각형 회전 토글
  // RotationSpeed 컴포넌트 유무로 회전 on/off 를 CommandBuffer 로 처리
  ImGui::SameLine();
  auto triView = ECS.view<TriangleShape, PlaySceneTag>();
  for (auto [e, ts] : triView.each()) {
    bool isRotating = ECS.all_of<RotationSpeed>(e);
    auto &cmb = ECS.ctx().get<PlayCmdBuffer>();
    if (isRotating) {
      if (ImGui::Button("Stop Rotation", ImVec2(160.0f, 32.0f)))
        cmb.Add(CmdRemoveRotationSpeed{e});
    } else {
      if (ImGui::Button("Start Rotation", ImVec2(160.0f, 32.0f)))
        cmb.Add(CmdAddRotationSpeed{e, RotationSpeed{1.26f}});
    }
    break;  // 삼각형 entity 는 하나
  }

  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(3);
}

}  // namespace

// =============================================================================
// Scene Hooks
// =============================================================================

/**
 * @brief Play 씬 진입 시 호출된다.
 *
 * CommandBuffer 를 ctx 에 등록하고, PlaySceneTag 가 붙은 entity 를 생성한다.
 *
 * 생성 Entity:
 * - 삼각형 : Transform + RotationSpeed + TriangleShape + Color(Red) + PlaySceneTag
 * - 원     : Transform + CircleShape + ScoreCounter + Color(Green) + PlaySceneTag
 * - 사각형 : Transform + RectShape + Color(Blue) + PlaySceneTag
 *
 * @param ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnEnter(entt::registry &ECS, float dt) {
  OnGenericSceneEnter(ECS, SceneId::Play);

  auto &lc = ECS.ctx().get<SceneLoadingContext>();

  // 단계 1: 텍스처 로딩 시뮬레이션
  lc.szStatus.store("텍스처 로딩 중...");
  lc.fProgress.store(0.0f);
  SDL_Delay(500);

  // CommandBuffer ctx 등록
  ECS.ctx().emplace<PlayCmdBuffer>();
  auto &cmb = ECS.ctx().get<PlayCmdBuffer>();

  // 단계 2: 오디오 리소스 로딩 시뮬레이션
  lc.szStatus.store("오디오 리소스 로딩 중...");
  lc.fProgress.store(0.25f);
  SDL_Delay(500);

  int winW = 0, winH = 0;
  SDL_GetWindowSize(ECS.ctx().get<AppCtx>().pWindow, &winW, &winH);
  const float triR = (float)(winH / 3) / 4.0f;

  // 삼각형 entity
  cmb.Add(CmdCreateEntity{[triR](entt::registry &reg, entt::entity e) {
    reg.emplace<Transform>(e, 1.0f / 6.0f, 0.5f, 0.0f);
    reg.emplace<RotationSpeed>(e, 1.26f);
    reg.emplace<TriangleShape>(e, triR);
    reg.emplace<Color>(e, (uint8_t)220, (uint8_t)50, (uint8_t)50);
    reg.emplace<PlaySceneTag>(e);
  }});

  // 단계 3: 엔티티 생성 시뮬레이션
  lc.szStatus.store("게임 오브젝트 생성 중...");
  lc.fProgress.store(0.5f);
  SDL_Delay(500);

  // 원 entity (ScoreCounter 겸용)
  cmb.Add(CmdCreateEntity{[](entt::registry &reg, entt::entity e) {
    reg.emplace<Transform>(e, 0.5f, 0.5f, 0.0f);
    reg.emplace<CircleShape>(e, 60.0f, 60.0f);
    reg.emplace<ScoreCounter>(e);
    reg.emplace<Color>(e, (uint8_t)50, (uint8_t)220, (uint8_t)50);
    reg.emplace<PlaySceneTag>(e);
  }});

  // 사각형 entity
  cmb.Add(CmdCreateEntity{[](entt::registry &reg, entt::entity e) {
    reg.emplace<Transform>(e, 5.0f / 6.0f, 0.5f, 0.0f);
    reg.emplace<RectShape>(e, 60.0f);
    reg.emplace<Color>(e, (uint8_t)50, (uint8_t)50, (uint8_t)220);
    reg.emplace<PlaySceneTag>(e);
  }});

  // 단계 4: 씬 초기화 시뮬레이션
  lc.szStatus.store("씬 초기화 중...");
  lc.fProgress.store(0.75f);
  SDL_Delay(500);

  // OnEnter 에서 즉시 Flush — 이 씬의 초기 entity 를 레지스트리에 반영
  cmb.Flush(ECS);

  // 완료
  lc.szStatus.store("로딩 완료!");
  lc.fProgress.store(1.0f);
}

/**
 * @brief Play 씬 이탈 시 호출된다.
 *
 * PlaySceneTag entity 를 일괄 destroy 하고 CommandBuffer ctx 를 해제한다.
 *
 * @param ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnExit(entt::registry &ECS, float dt) {
  OnGenericSceneExit(ECS, SceneId::Play);

  auto &lc = ECS.ctx().get<SceneLoadingContext>();

  // 단계 1: 게임 상태 저장 시뮬레이션
  lc.szStatus.store("게임 상태 저장 중...");
  lc.fProgress.store(0.0f);
  SDL_Delay(500);

  // 씬 소유 entity 정리
  auto view = ECS.view<PlaySceneTag>();
  ECS.destroy(view.begin(), view.end());

  // 단계 2: 오디오 리소스 해제 시뮬레이션
  lc.szStatus.store("오디오 리소스 해제 중...");
  lc.fProgress.store(0.5f);
  SDL_Delay(500);

  // CommandBuffer ctx 해제
  ECS.ctx().erase<PlayCmdBuffer>();

  // 완료
  lc.szStatus.store("정리 완료!");
  lc.fProgress.store(1.0f);
}

/**
 * @brief 매 프레임 게임 로직을 갱신한다.
 *
 * 각 System 은 CommandBuffer 에 변경 의도만 Submit 하고,
 * 마지막 Flush() 에서 일괄 반영한다.
 *
 * System 호출 순서:
 *  1. ScoreSystem       — tick/score 변경 예약 (PulseSystem 이 tick 스냅샷을 읽으므로 먼저)
 *  2. RotationSystem    — Transform.angle 변경 예약
 *  3. PulseSystem       — CircleShape.r 변경 예약 (tick 스냅샷 기준)
 *  4. GlobalStateSystem — GameState ctx 변경 예약
 *  5. InputSystem       — 좌우 방향키로 사각형 angle 변경 예약
 *  6. Flush             — 이 턴의 모든 변경 일괄 반영
 *
 * @param ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnUpdate(entt::registry &ECS, float dt) {
  ScoreSystem(ECS, dt);
  RotationSystem(ECS, dt);
  PulseSystem(ECS, dt);
  GlobalStateSystem(ECS, dt);
  InputSystem(ECS, dt);
  ECS.ctx().get<PlayCmdBuffer>().Flush(ECS);
}

/**
 * @brief 매 프레임 렌더링을 수행한다. 레지스트리는 읽기 전용으로만 접근한다.
 *
 * 렌더링 순서 (SDL 먼저, ImGui 나중):
 *  1. 게임 영역 배경
 *  2. RenderTriangleSystem
 *  3. RenderCircleSystem
 *  4. RenderRectSystem
 *  5. HudSystem (ImGui)
 *
 * @param ECS ECS 레지스트리
 * @param[in] dt  프레임 경과 시간 (초, OnEnter/OnExit 는 0.0f)
 */
void OnRender(entt::registry &ECS, float dt) {
  auto &appCtx = ECS.ctx().get<AppCtx>();

  // 게임 영역 배경
  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);
  const int gameY = winH / 3;
  const int gameH = winH - gameY;
  SDL_SetRenderDrawColor(appCtx.pRenderer, 40, 40, 40, SDL_ALPHA_OPAQUE);
  SDL_Rect gameArea{0, gameY, winW, gameH};
  SDL_RenderFillRect(appCtx.pRenderer, &gameArea);

  RenderTriangleSystem(ECS, dt);
  RenderCircleSystem(ECS, dt);
  RenderRectSystem(ECS, dt);
  HudSystem(ECS, dt);
}

}  // namespace Scene::Play
