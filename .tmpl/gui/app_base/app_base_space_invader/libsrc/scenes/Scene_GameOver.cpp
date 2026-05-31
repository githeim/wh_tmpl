#include "Scene_GameOver.h"
#include "SceneUtil.h"
#include "SceneDef.h"
#include "AppState.h"
#include "SDL2_Ctx.h"
#include "imgui.h"

#include <SDL2/SDL.h>

namespace Scene::GameOver {

void OnEnter(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneEnter(ECS, SceneId::GameOver);
}

void OnExit(entt::registry &ECS, float dt) {
  (void)dt;
  OnGenericSceneExit(ECS, SceneId::GameOver);
}

void OnRender(entt::registry &ECS, float dt) {
  (void)dt;
  auto &appCtx     = ECS.ctx().get<SDL2Ctx>();
  auto &dispatcher = ECS.ctx().get<entt::dispatcher>();
  const auto &input = ECS.ctx().get<InputState>();
  const auto &gs    = ECS.ctx().get<GameState>();

  int winW = 0, winH = 0;
  SDL_GetWindowSize(appCtx.pWindow, &winW, &winH);

  // 배경
  SDL_SetRenderDrawColor(appCtx.pRenderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  SDL_Rect bg{0, 0, winW, winH};
  SDL_RenderFillRect(appCtx.pRenderer, &bg);

  // ImGui 오버레이
  BeginFullscreenUi("GameOverUI");

  ImGuiViewport *vp = ImGui::GetMainViewport();
  float cx = vp->Size.x * 0.5f;
  float cy = vp->Size.y * 0.38f;

  ImGui::SetWindowFontScale(3.0f);
  ImGui::SetCursorPos(ImVec2(cx - 130.0f, cy));
  ImGui::TextUnformatted("Game Over");

  ImGui::SetWindowFontScale(1.8f);
  ImGui::SetCursorPos(ImVec2(cx - 100.0f, cy + 90.0f));
  ImGui::Text("Score: %d", gs.score);

  ImGui::SetWindowFontScale(1.3f);
  ImGui::SetCursorPos(ImVec2(cx - 90.0f, cy + 150.0f));
  ImGui::TextUnformatted("space to title");

  ImGui::SetWindowFontScale(1.0f);

  EndFullscreenUi();

  // 입력
  if (input.IsKeyHeld(SDLK_SPACE)) {
    dispatcher.enqueue<SceneTransitionRequest>(SceneTransitionRequest{SceneId::Title});
  }
}

}  // namespace Scene::GameOver
