#include "Scene_Game.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "SDL2_Ctx.h"
#include "ECSUtil.h"
#include "imgui.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <cstdlib>
#include <ctime>

namespace Scene::Game {

namespace {

// =============================================================================
// 상수
// =============================================================================

constexpr int   kEnemyCols      = 8;
constexpr int   kEnemyRows      = 4;
constexpr int   kEnemyCount     = kEnemyCols * kEnemyRows;
constexpr float kEnemyShootMin  = 1.0f;   ///< 최소 사격 간격 (초)
constexpr float kEnemyShootMax  = 3.0f;   ///< 최대 사격 간격 (초)
constexpr float kEnemyMoveSpeed = 60.0f;  ///< 적기 이동 속도 (px/s)
constexpr float kPlayerSpeed    = 280.0f; ///< 아군기 이동 속도 (px/s)
constexpr float kBulletSpeed    = 400.0f; ///< 총알 속도 (px/s)
constexpr int   kPlayerShipSize = 28;
constexpr int   kEnemyW         = 32;     ///< 적기 폭
constexpr int   kEnemyH         = 20;     ///< 적기 높이
constexpr int   kBulletW        = 4;
constexpr int   kBulletH        = 12;
constexpr int   kScorePerKill   = 10;

// =============================================================================
// Components
// =============================================================================

struct Position  { float x, y; };
struct Velocity  { float vx, vy; };
struct Size      { int w, h; };

struct PlayerTag  {};
struct EnemyTag   {
  float shootTimer = 0.0f;  ///< 다음 사격까지 남은 시간 (적기마다 독립)
};

/// 총알 진행 방향: Up = 아군 총알, Down = 적 총알
enum class BulletDir { Up, Down };
struct BulletTag  { BulletDir dir; };

/// 이번 프레임에 제거 예약된 entity
struct DeadTag    {};

/// 게임 씬 전용 tag (OnExit 시 일괄 정리)
struct GameSceneTag {};

// =============================================================================
// Commands
// =============================================================================

struct CmdUpdatePosition { entt::entity e; Position value; };
struct CmdUpdateVelocity { entt::entity e; Velocity value; };
struct CmdUpdateEnemyTag { entt::entity e; EnemyTag value; };

inline void Apply(entt::registry &reg, const CmdUpdatePosition &c) {
  if (reg.valid(c.e) && reg.all_of<Position>(c.e))
    reg.get<Position>(c.e) = c.value;
}
inline void Apply(entt::registry &reg, const CmdUpdateVelocity &c) {
  if (reg.valid(c.e) && reg.all_of<Velocity>(c.e))
    reg.get<Velocity>(c.e) = c.value;
}
inline void Apply(entt::registry &reg, const CmdUpdateEnemyTag &c) {
  if (reg.valid(c.e) && reg.all_of<EnemyTag>(c.e))
    reg.get<EnemyTag>(c.e) = c.value;
}

using GameCmd = std::variant<
  CmdDestroyEntity,
  CmdCreateEntity,
  CmdSubmit,
  CmdUpdatePosition,
  CmdUpdateVelocity,
  CmdUpdateEnemyTag
>;
using GameCmdBuffer = CommandBuffer<GameCmd>;

// =============================================================================
// 게임 상태 (씬 로컬 ctx)
// =============================================================================

struct GameLocalState {
  float enemyDirX    = 1.0f;   ///< 적기 이동 방향 (+1 오른쪽, -1 왼쪽)
  bool  gameOver     = false;
  bool  spaceWasHeld = false;  ///< 스페이스 연사 방지용
};

// =============================================================================
// SDL 도형 헬퍼
// =============================================================================

static void DrawFilledTriangle(SDL_Renderer *r,
                               int x1, int y1,
                               int x2, int y2,
                               int x3, int y3) {
  auto sort2 = [](int &ax, int &ay, int &bx, int &by) {
    if (ay > by) { std::swap(ax, bx); std::swap(ay, by); }
  };
  sort2(x1, y1, x2, y2);
  sort2(x1, y1, x3, y3);
  sort2(x2, y2, x3, y3);

  float s1 = (y2 - y1) ? (float)(x2 - x1) / (y2 - y1) : 0.0f;
  float s2 = (y3 - y1) ? (float)(x3 - x1) / (y3 - y1) : 0.0f;
  float s3 = (y3 - y2) ? (float)(x3 - x2) / (y3 - y2) : 0.0f;

  for (int y = y1; y <= y2; ++y) {
    int xl = (int)(x1 + s1 * (y - y1));
    int xr = (int)(x1 + s2 * (y - y1));
    if (xl > xr) std::swap(xl, xr);
    SDL_RenderDrawLine(r, xl, y, xr, y);
  }
  for (int y = y2; y <= y3; ++y) {
    int xl = (int)(x2 + s3 * (y - y2));
    int xr = (int)(x1 + s2 * (y - y1));
    if (xl > xr) std::swap(xl, xr);
    SDL_RenderDrawLine(r, xl, y, xr, y);
  }
}

static void DrawFilledRect(SDL_Renderer *r, int x, int y, int w, int h) {
  SDL_Rect rect{x, y, w, h};
  SDL_RenderFillRect(r, &rect);
}

/// 아군기 드로잉 (Scene_Title 과 동일한 모양, size 파라미터로 크기 조절)
static void DrawPlayerShip(SDL_Renderer *r, int cx, int cy, int size) {
  int s = size;
  // 엔진 불꽃
  SDL_SetRenderDrawColor(r, 255, 140, 0, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r,
      cx, cy + s / 2 + s / 4,
      cx - s / 6, cy + s / 2,
      cx + s / 6, cy + s / 2);
  // 몸통
  SDL_SetRenderDrawColor(r, 100, 220, 255, SDL_ALPHA_OPAQUE);
  DrawFilledRect(r, cx - s / 8, cy - s / 2, s / 4, s);
  // 날개
  SDL_SetRenderDrawColor(r, 60, 180, 220, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r, cx, cy + s / 6, cx - s / 2, cy + s / 2, cx - s / 8, cy - s / 6);
  DrawFilledTriangle(r, cx, cy + s / 6, cx + s / 2, cy + s / 2, cx + s / 8, cy - s / 6);
  // 앞부리
  SDL_SetRenderDrawColor(r, 240, 240, 255, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r, cx, cy - s / 2, cx - s / 8, cy - s / 6, cx + s / 8, cy - s / 6);
  // 캐노피
  SDL_SetRenderDrawColor(r, 30, 80, 160, SDL_ALPHA_OPAQUE);
  DrawFilledRect(r, cx - s / 12, cy - s / 4, s / 6, s / 5);
}

/// 적기 드로잉: 간단한 역삼각형 + 더듬이 형태
static void DrawEnemy(SDL_Renderer *r, int cx, int cy, int w, int h) {
  // 몸통 (역삼각형)
  SDL_SetRenderDrawColor(r, 200, 50, 50, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r,
      cx,         cy - h / 2,
      cx - w / 2, cy + h / 2,
      cx + w / 2, cy + h / 2);
  // 더듬이 (왼쪽)
  SDL_SetRenderDrawColor(r, 255, 100, 100, SDL_ALPHA_OPAQUE);
  DrawFilledRect(r, cx - w / 2, cy - h / 2 - h / 4, 3, h / 4);
  // 더듬이 (오른쪽)
  DrawFilledRect(r, cx + w / 2 - 3, cy - h / 2 - h / 4, 3, h / 4);
}

// =============================================================================
// Systems — Update
// =============================================================================

/// 아군기 좌우 이동 입력 처리
static void PlayerInputSystem(entt::registry &ECS, float dt) {
  const auto &input = ECS.ctx().get<InputState>();
  auto &cmb         = ECS.ctx().get<GameCmdBuffer>();
  auto &dispatcher  = ECS.ctx().get<entt::dispatcher>();
  auto &appCtx      = ECS.ctx().get<SDL2Ctx>();
  auto &ls          = ECS.ctx().get<GameLocalState>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  for (auto [e, pos, vel] : ECS.view<Position, Velocity, PlayerTag>().each()) {
    Velocity nextVel{0.0f, 0.0f};
    if (input.IsKeyHeld(SDLK_LEFT))  nextVel.vx = -kPlayerSpeed;
    if (input.IsKeyHeld(SDLK_RIGHT)) nextVel.vx =  kPlayerSpeed;
    cmb.Add(CmdUpdateVelocity{e, nextVel});

    // 스페이스바: 총알 발사 (연사 방지 — 뗐다 눌러야 재발사)
    bool spaceNow = input.IsKeyHeld(SDLK_SPACE);
    if (spaceNow && !ls.spaceWasHeld) {
      float bx = pos.x;
      float by = pos.y - kPlayerShipSize / 2.0f;
      cmb.Add(CmdCreateEntity{[bx, by](entt::registry &reg, entt::entity ne) {
        reg.emplace<Position>(ne, bx, by);
        reg.emplace<Velocity>(ne, 0.0f, -kBulletSpeed);
        reg.emplace<Size>(ne, kBulletW, kBulletH);
        reg.emplace<BulletTag>(ne, BulletDir::Up);
        reg.emplace<GameSceneTag>(ne);
      }});
    }
    cmb.Add(CmdSubmit{[spaceNow](entt::registry &reg) {
      reg.ctx().get<GameLocalState>().spaceWasHeld = spaceNow;
    }});

    // ESC → Title
    if (input.IsKeyHeld(SDLK_ESCAPE)) {
      dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::Title});
    }
    break;
  }
}

/// 모든 velocity entity 위치 갱신, 화면 밖 총알 제거
static void MoveSystem(entt::registry &ECS, float dt) {
  auto &cmb    = ECS.ctx().get<GameCmdBuffer>();
  auto &appCtx = ECS.ctx().get<SDL2Ctx>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // 아군기 이동 + 벽 클램프
  for (auto [e, pos, vel] : ECS.view<Position, Velocity, PlayerTag>().each()) {
    Position next{pos.x + vel.vx * dt, pos.y};
    float half = kPlayerShipSize / 2.0f;
    if (next.x < half)           next.x = half;
    if (next.x > winW - half)    next.x = (float)(winW) - half;
    cmb.Add(CmdUpdatePosition{e, next});
  }

  // 총알 이동 + 화면 밖 제거
  for (auto [e, pos, vel, bt] : ECS.view<Position, Velocity, BulletTag>().each()) {
    Position next{pos.x + vel.vx * dt, pos.y + vel.vy * dt};
    cmb.Add(CmdUpdatePosition{e, next});
    if (next.y < -kBulletH || next.y > winH + kBulletH) {
      cmb.Add(CmdDestroyEntity{e});
    }
  }
}

/// 적기 집단 좌우 이동 (벽 반사)
static void EnemyMoveSystem(entt::registry &ECS, float dt) {
  auto &ls     = ECS.ctx().get<GameLocalState>();
  auto &cmb    = ECS.ctx().get<GameCmdBuffer>();
  auto &appCtx = ECS.ctx().get<SDL2Ctx>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // 이번 프레임 이동량
  float dx = kEnemyMoveSpeed * ls.enemyDirX * dt;

  // 이동 후 좌우 끝 좌표 계산
  float minX =  1e9f, maxX = -1e9f;
  for (auto [e, pos, et] : ECS.view<Position, EnemyTag>().each()) {
    float nx = pos.x + dx;
    if (nx < minX) minX = nx;
    if (nx > maxX) maxX = nx;
  }

  // 벽 충돌 시 방향 반전
  bool reverse = (minX < kEnemyW / 2.0f && ls.enemyDirX < 0) ||
                 (maxX > winW - kEnemyW / 2.0f && ls.enemyDirX > 0);
  if (reverse) {
    dx = 0.0f;  // 반전 프레임은 이동 없음
    cmb.Add(CmdSubmit{[](entt::registry &reg) {
      reg.ctx().get<GameLocalState>().enemyDirX *= -1.0f;
    }});
  }

  if (dx == 0.0f) return;
  for (auto [e, pos, et] : ECS.view<Position, EnemyTag>().each()) {
    cmb.Add(CmdUpdatePosition{e, {pos.x + dx, pos.y}});
  }
}

/// 각 적기 개별 사격 타이머 처리
static void EnemyShootSystem(entt::registry &ECS, float dt) {
  auto &cmb = ECS.ctx().get<GameCmdBuffer>();

  for (auto [e, pos, et] : ECS.view<Position, EnemyTag>().each()) {
    EnemyTag next = et;
    next.shootTimer -= dt;
    if (next.shootTimer <= 0.0f) {
      // 총알 발사
      float bx = pos.x;
      float by = pos.y + kEnemyH / 2.0f;
      cmb.Add(CmdCreateEntity{[bx, by](entt::registry &reg, entt::entity ne) {
        reg.emplace<Position>(ne, bx, by);
        reg.emplace<Velocity>(ne, 0.0f, kBulletSpeed);
        reg.emplace<Size>(ne, kBulletW, kBulletH);
        reg.emplace<BulletTag>(ne, BulletDir::Down);
        reg.emplace<GameSceneTag>(ne);
      }});
      // 다음 사격 딜레이 갱신 (1~3초 랜덤)
      next.shootTimer = kEnemyShootMin +
          (float)(rand() % 1000) / 1000.0f * (kEnemyShootMax - kEnemyShootMin);
    }
    cmb.Add(CmdUpdateEnemyTag{e, next});
  }
}

/// 총알 충돌 처리
static void CollisionSystem(entt::registry &ECS, float) {
  auto &cmb      = ECS.ctx().get<GameCmdBuffer>();
  auto &appCtx   = ECS.ctx().get<SDL2Ctx>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  auto overlap = [](float ax, float ay, int aw, int ah,
                    float bx, float by, int bw, int bh) {
    return ax - aw / 2 < bx + bw / 2 && ax + aw / 2 > bx - bw / 2 &&
           ay - ah / 2 < by + bh / 2 && ay + ah / 2 > by - bh / 2;
  };

  // 총알끼리 충돌 (Up vs Down)
  std::vector<entt::entity> upBullets, downBullets;
  for (auto [e, bt] : ECS.view<BulletTag>().each()) {
    if (bt.dir == BulletDir::Up)   upBullets.push_back(e);
    else                            downBullets.push_back(e);
  }
  for (auto ue : upBullets) {
    if (!ECS.valid(ue)) continue;
    auto &up = ECS.get<Position>(ue);
    for (auto de : downBullets) {
      if (!ECS.valid(de)) continue;
      auto &dp = ECS.get<Position>(de);
      if (overlap(up.x, up.y, kBulletW, kBulletH,
                  dp.x, dp.y, kBulletW, kBulletH)) {
        cmb.Add(CmdDestroyEntity{ue});
        cmb.Add(CmdDestroyEntity{de});
        break;
      }
    }
  }

  // 아군 총알 vs 적기
  for (auto ue : upBullets) {
    if (!ECS.valid(ue)) continue;
    auto &up = ECS.get<Position>(ue);
    for (auto [ee, epos, eet] : ECS.view<Position, EnemyTag>().each()) {
      if (overlap(up.x, up.y, kBulletW, kBulletH,
                  epos.x, epos.y, kEnemyW, kEnemyH)) {
        cmb.Add(CmdDestroyEntity{ue});
        cmb.Add(CmdDestroyEntity{ee});
        // 점수 증가
        cmb.Add(CmdSubmit{[](entt::registry &reg) {
          reg.ctx().get<GameState>().score += kScorePerKill;
        }});
        break;
      }
    }
  }

  // 적 총알 vs 아군기
  for (auto [pe, ppos] : ECS.view<Position, PlayerTag>().each()) {
    for (auto de : downBullets) {
      if (!ECS.valid(de)) continue;
      auto &dp = ECS.get<Position>(de);
      if (overlap(dp.x, dp.y, kBulletW, kBulletH,
                  ppos.x, ppos.y, kPlayerShipSize, kPlayerShipSize)) {
        cmb.Add(CmdDestroyEntity{de});
        cmb.Add(CmdSubmit{[](entt::registry &reg) {
          auto &gs = reg.ctx().get<GameState>();
          gs.playerLives--;
        }});
        break;
      }
    }
  }
}

/// 게임 오버 조건 확인 (적기 전멸 or 목숨 0)
static void GameOverCheckSystem(entt::registry &ECS, float) {
  auto &ls         = ECS.ctx().get<GameLocalState>();
  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  auto &gs         = ECS.ctx().get<GameState>();

  if (ls.gameOver) return;

  bool noEnemies = ECS.view<EnemyTag>().begin() == ECS.view<EnemyTag>().end();
  bool noLives   = gs.playerLives <= 0;

  if (noEnemies || noLives) {
    ls.gameOver = true;
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::GameOver});
  }
}

// =============================================================================
// Render
// =============================================================================

static void RenderGame(entt::registry &ECS, float) {
  auto &appCtx = ECS.ctx().get<SDL2Ctx>();
  auto  r      = appCtx.pRenderer;
  auto &gs     = ECS.ctx().get<GameState>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // 배경
  SDL_SetRenderDrawColor(r, 5, 5, 20, SDL_ALPHA_OPAQUE);
  SDL_Rect bg{0, 0, winW, winH};
  SDL_RenderFillRect(r, &bg);

  // 적기
  for (auto [e, pos, et] : ECS.view<Position, EnemyTag>().each()) {
    DrawEnemy(r, (int)pos.x, (int)pos.y, kEnemyW, kEnemyH);
  }

  // 아군기
  for (auto [e, pos] : ECS.view<Position, PlayerTag>().each()) {
    DrawPlayerShip(r, (int)pos.x, (int)pos.y, kPlayerShipSize);
  }

  // 총알
  for (auto [e, pos, bt] : ECS.view<Position, BulletTag>().each()) {
    if (bt.dir == BulletDir::Up)
      SDL_SetRenderDrawColor(r, 100, 255, 100, SDL_ALPHA_OPAQUE);
    else
      SDL_SetRenderDrawColor(r, 255, 80, 80, SDL_ALPHA_OPAQUE);
    DrawFilledRect(r, (int)pos.x - kBulletW / 2, (int)pos.y - kBulletH / 2,
                   kBulletW, kBulletH);
  }

  // HUD (ImGui)
  BeginFullscreenUi("GameHUD");
  ImGui::SetWindowFontScale(1.2f);
  ImGui::Text("Score: %d    Lives: %d", gs.score, gs.playerLives);
  ImGui::SetWindowFontScale(1.0f);
  EndFullscreenUi();
}

}  // namespace

// =============================================================================
// Scene Hooks
// =============================================================================

void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::Game);

  srand((unsigned int)time(nullptr));

  // GameState 리셋
  ECS.ctx().get<GameState>().score       = 0;
  ECS.ctx().get<GameState>().playerLives = 3;

  // CommandBuffer & LocalState
  ECS.ctx().emplace<GameCmdBuffer>();
  ECS.ctx().emplace<GameLocalState>();

  auto &appCtx = ECS.ctx().get<SDL2Ctx>();
  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // 아군기 생성 (하단 중앙)
  float px = winW / 2.0f;
  float py = winH - 60.0f;
  auto player = ECS.create();
  ECS.emplace<Position>(player, px, py);
  ECS.emplace<Velocity>(player, 0.0f, 0.0f);
  ECS.emplace<PlayerTag>(player);
  ECS.emplace<GameSceneTag>(player);

  // 적기 8x4 배치
  float marginX  = winW * 0.1f;
  float marginY  = winH * 0.08f;
  float spacingX = (winW - marginX * 2.0f) / (kEnemyCols - 1);
  float spacingY = 50.0f;

  for (int row = 0; row < kEnemyRows; ++row) {
    for (int col = 0; col < kEnemyCols; ++col) {
      float ex = marginX + col * spacingX;
      float ey = marginY + row * spacingY;
      auto e = ECS.create();
      ECS.emplace<Position>(e, ex, ey);
      // 초기 사격 타이머: 1~3초 랜덤 (적기마다 다른 시점에 첫 발사)
      float initTimer = kEnemyShootMin +
          (float)(rand() % 1000) / 1000.0f * (kEnemyShootMax - kEnemyShootMin);
      ECS.emplace<EnemyTag>(e, initTimer);
      ECS.emplace<GameSceneTag>(e);
    }
  }
}

void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::Game);

  // GameSceneTag entity 전부 제거
  auto view = ECS.view<GameSceneTag>();
  ECS.destroy(view.begin(), view.end());

  ECS.ctx().erase<GameCmdBuffer>();
  ECS.ctx().erase<GameLocalState>();
}

void OnUpdate(entt::registry &ECS, float dt) {
  PlayerInputSystem(ECS, dt);
  EnemyMoveSystem(ECS, dt);
  MoveSystem(ECS, dt);
  EnemyShootSystem(ECS, dt);
  CollisionSystem(ECS, dt);
  GameOverCheckSystem(ECS, dt);
  ECS.ctx().get<GameCmdBuffer>().Flush(ECS);
}

void OnRender(entt::registry &ECS, float dt) {
  RenderGame(ECS, dt);
}

}  // namespace Scene::Game
