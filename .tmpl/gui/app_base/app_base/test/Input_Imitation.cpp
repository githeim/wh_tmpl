#include "Input_Imitation.h"

#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>

#include <SDL2/SDL.h>

#include "SceneDef.h"   // WidgetRegistry_T, ExtEvt_T
#include "SDL2_Ctx.h"   // SDL2Ctx (pWindow 접근용)

// 클릭 edge 회피 offset
#define X_OFFSET (5)
#define Y_OFFSET (5)

// TEST_VISUAL=1 이면 true
static bool IsVisualMode() {
  const char *v = std::getenv("TEST_VISUAL");
  return v != nullptr && v[0] == '1';
}

// ── 이벤트 생성 헬퍼 ──────────────────────────────────────────────────────────

void CreateMouseMotionEvt(SDL_Event *pEvt, int32_t iX, int32_t iY) {
  pEvt->type = SDL_MOUSEMOTION;
  SDL_MouseMotionEvent evtMotion = {.type = SDL_MOUSEMOTION, .x = iX, .y = iY};
  memcpy(&pEvt->motion, &evtMotion, sizeof(evtMotion));
}

void CreateMouseButtonEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                          Uint32 iType, uint8_t iState, uint8_t iButton) {
  pEvt->type = iType;
  SDL_MouseButtonEvent evtBtn = {.type   = iType,
                                 .button = iButton,
                                 .state  = iState,
                                 .clicks = 1,
                                 .x      = iX,
                                 .y      = iY};
  memcpy(pEvt, &evtBtn, sizeof(evtBtn));
}

void CreateMouseButtonDownEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                              uint8_t iButton) {
  CreateMouseButtonEvt(pEvt, iX, iY, SDL_MOUSEBUTTONDOWN, SDL_PRESSED, iButton);
}

void CreateMouseButtonUpEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                            uint8_t iButton) {
  CreateMouseButtonEvt(pEvt, iX, iY, SDL_MOUSEBUTTONUP, SDL_RELEASED, iButton);
}

// ── Visual 모드 전용: 커서를 단계적으로 이동시킨다 ──────────────────────────

/**
 * @brief 현재 커서 위치에서 목표 (gX, gY) 까지 step 개 구간으로 나눠 이동한다.
 *
 * @param gX     전역 목표 X (SDL_WarpMouseGlobal 기준)
 * @param gY     전역 목표 Y
 * @param steps  이동 단계 수
 * @param delay  단계 간 대기 시간 (ms)
 */
static void WarpMouseSmooth(int gX, int gY, int steps = 20, int delay_ms = 15) {
  int curX, curY;
  SDL_GetGlobalMouseState(&curX, &curY);

  for (int i = 1; i <= steps; ++i) {
    int mx = curX + (gX - curX) * i / steps;
    int my = curY + (gY - curY) * i / steps;
    SDL_WarpMouseGlobal(mx, my);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

// ── Click_Evt (좌표 직접) ─────────────────────────────────────────────────────

void Click_Evt(std::shared_ptr<entt::registry> &pECS,
               int32_t iX, int32_t iY, uint8_t iButton) {
  if (!pECS) return;
  if (!pECS->ctx().contains<ExtEvt_T>()) return;

  auto &extEvt = pECS->ctx().get<ExtEvt_T>();
  const int tx = iX + X_OFFSET;
  const int ty = iY + Y_OFFSET;

  if (IsVisualMode()) {
    // ── Visual 모드 ──────────────────────────────────────────────────────────
    // 윈도우 전역 좌표 = 윈도우 위치 + 클라이언트 좌표
    int winX = 0, winY = 0;
    if (pECS->ctx().contains<SDL2Ctx>()) {
      auto &sdlCtx = pECS->ctx().get<SDL2Ctx>();
      if (sdlCtx.pWindow) {
        SDL_GetWindowPosition(sdlCtx.pWindow, &winX, &winY);
      }
    }

    // 커서를 부드럽게 이동
    WarpMouseSmooth(winX + tx, winY + ty);

    // hover 판정을 위해 ImGui 에 2프레임 이상 여유를 준다 (~50ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ButtonDown/Up 만 주입 (위치는 ImGui 가 GlobalMouseState 로 읽음)
    SDL_Event evtDown, evtUp;
    CreateMouseButtonDownEvt(&evtDown, tx, ty, iButton);
    CreateMouseButtonUpEvt  (&evtUp,   tx, ty, iButton);
    extEvt.vecEvents.push_back(evtDown);
    extEvt.vecEvents.push_back(evtUp);

    // 클릭 후 잠시 대기 (시각적으로 확인 가능하게)
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

  } else {
    // ── Offscreen 모드 ───────────────────────────────────────────────────────
    SDL_Event evtMotion, evtDown, evtUp;
    CreateMouseMotionEvt    (&evtMotion, tx, ty);
    CreateMouseButtonDownEvt(&evtDown,   tx, ty, iButton);
    CreateMouseButtonUpEvt  (&evtUp,     tx, ty, iButton);

    extEvt.vecEvents.push_back(evtMotion);
    extEvt.vecEvents.push_back(evtDown);
    extEvt.vecEvents.push_back(evtUp);
  }
}

// ── Click_Evt (위젯 이름) ─────────────────────────────────────────────────────

bool Click_Evt(std::shared_ptr<entt::registry> &pECS,
               const std::string &funcName, const std::string &itemName,
               uint8_t iButton) {
  if (!pECS) return false;
  if (!pECS->ctx().contains<WidgetRegistry_T>()) return false;

  const auto &reg = pECS->ctx().get<WidgetRegistry_T>();
  const std::string key = funcName + ":" + itemName;
  auto it = reg.map.find(key);
  if (it == reg.map.end()) return false;

  Click_Evt(pECS, it->second.iX, it->second.iY, iButton);
  return true;
}

