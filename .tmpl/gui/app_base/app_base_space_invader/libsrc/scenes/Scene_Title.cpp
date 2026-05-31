#include "Scene_Title.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "SDL2_Ctx.h"
#include "imgui.h"

#include <SDL2/SDL.h>
#include <cmath>

namespace Scene::Title {

// =============================================================================
// SDL 도형 헬퍼 (아군기 드로잉)
// =============================================================================

/**
 * @brief SDL Renderer 로 속이 찬 삼각형을 그린다. (scan-line 방식)
 */
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

  auto scanLine = [&](int y, float lx, float rx) {
    int xl = static_cast<int>(lx), xr = static_cast<int>(rx);
    if (xl > xr) std::swap(xl, xr);
    SDL_RenderDrawLine(r, xl, y, xr, y);
  };

  float s1 = (y2 - y1) ? (float)(x2 - x1) / (y2 - y1) : 0.0f;
  float s2 = (y3 - y1) ? (float)(x3 - x1) / (y3 - y1) : 0.0f;
  float s3 = (y3 - y2) ? (float)(x3 - x2) / (y3 - y2) : 0.0f;

  for (int y = y1; y <= y2; ++y)
    scanLine(y, x1 + s1 * (y - y1), x1 + s2 * (y - y1));
  for (int y = y2; y <= y3; ++y)
    scanLine(y, x2 + s3 * (y - y2), x1 + s2 * (y - y1));
}

static void DrawFilledRect(SDL_Renderer *r, int x, int y, int w, int h) {
  SDL_Rect rect{x, y, w, h};
  SDL_RenderFillRect(r, &rect);
}

/**
 * @brief 아군 전투기를 cx, cy 중심으로 size 크기로 그린다.
 *
 *  도형 구성:
 *   - 기체 몸통: 세로 직사각형
 *   - 주날개:    가로 넓은 삼각형 (좌우 대칭)
 *   - 앞부리:    위를 향한 뾰족한 삼각형
 *   - 엔진 불꽃: 아래쪽 작은 삼각형 (주황)
 */
static void DrawPlayerShip(SDL_Renderer *r, int cx, int cy, int size) {
  int s = size;

  // 엔진 불꽃 (주황)
  SDL_SetRenderDrawColor(r, 255, 140, 0, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r,
      cx,          cy + s / 2 + s / 4,
      cx - s / 6,  cy + s / 2,
      cx + s / 6,  cy + s / 2);

  // 기체 몸통 (밝은 청록)
  SDL_SetRenderDrawColor(r, 100, 220, 255, SDL_ALPHA_OPAQUE);
  DrawFilledRect(r, cx - s / 8, cy - s / 2, s / 4, s);

  // 주날개 (청록)
  SDL_SetRenderDrawColor(r, 60, 180, 220, SDL_ALPHA_OPAQUE);
  // 왼쪽 날개
  DrawFilledTriangle(r,
      cx,          cy + s / 6,
      cx - s / 2,  cy + s / 2,
      cx - s / 8,  cy - s / 6);
  // 오른쪽 날개
  DrawFilledTriangle(r,
      cx,          cy + s / 6,
      cx + s / 2,  cy + s / 2,
      cx + s / 8,  cy - s / 6);

  // 앞부리 (흰색)
  SDL_SetRenderDrawColor(r, 240, 240, 255, SDL_ALPHA_OPAQUE);
  DrawFilledTriangle(r,
      cx,          cy - s / 2,
      cx - s / 8,  cy - s / 6,
      cx + s / 8,  cy - s / 6);

  // 조종석 캐노피 (어두운 파랑)
  SDL_SetRenderDrawColor(r, 30, 80, 160, SDL_ALPHA_OPAQUE);
  DrawFilledRect(r, cx - s / 12, cy - s / 4, s / 6, s / 5);
}

// =============================================================================
// Scene Hooks
// =============================================================================

void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::Title);
}

void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::Title);
}

/**
 * @brief Title 씬 렌더링.
 *
 * - 배경: 검정
 * - 중앙 상단: 아군기 크게 그림 (SDL)
 * - ImGui 오버레이: "Space Invader" / "space to start" / "esc to quit"
 */
void OnRender(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx = ECS.ctx().get<SDL2Ctx>();
  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  const auto &input = ECS.ctx().get<InputState>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // ── 배경 ──
  SDL_SetRenderDrawColor(appCtx.pRenderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  SDL_Rect bg{0, 0, winW, winH};
  SDL_RenderFillRect(appCtx.pRenderer, &bg);

  // ── 아군기 (화면 상단 40% 중앙) ──
  int shipSize = winH / 4;
  int shipCx   = winW / 2;
  int shipCy   = winH * 28 / 100;  // 약 28% 높이
  DrawPlayerShip(appCtx.pRenderer, shipCx, shipCy, shipSize);

  // ── ImGui 텍스트 오버레이 ──
  BeginFullscreenUi("TitleUI");

  // 타이틀
  ImGuiViewport *vp = ImGui::GetMainViewport();
  float centerX = vp->Size.x * 0.5f;
  float titleY  = vp->Size.y * 0.58f;

  ImGui::SetCursorPos(ImVec2(centerX - 160.0f, titleY));
  ImGui::SetWindowFontScale(2.8f);
  ImGui::TextUnformatted("Space Invader");

  ImGui::SetWindowFontScale(1.4f);
  ImGui::SetCursorPos(ImVec2(centerX - 80.0f, titleY + 70.0f));
  ImGui::TextUnformatted("space to start");

  ImGui::SetCursorPos(ImVec2(centerX - 60.0f, titleY + 105.0f));
  ImGui::TextUnformatted("esc to quit");

  ImGui::SetWindowFontScale(1.0f);

  EndFullscreenUi();

  // ── 입력 처리 ──
  if (input.IsKeyHeld(SDLK_SPACE)) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::Game});
  }
  if (input.IsKeyHeld(SDLK_ESCAPE)) {
    dispatcher.enqueue<AppQuitRequest>();
  }
}

}  // namespace Scene::Title
