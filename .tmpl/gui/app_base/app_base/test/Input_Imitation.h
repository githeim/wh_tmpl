#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <SDL2/SDL_events.h>
#include <entt/entt.hpp>

/**
 * @brief 마우스 이동 SDL_Event 를 생성한다.
 */
void CreateMouseMotionEvt(SDL_Event *pEvt, int32_t iX, int32_t iY);

/**
 * @brief 마우스 버튼 SDL_Event 를 생성한다.
 */
void CreateMouseButtonEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                          Uint32 iType, uint8_t iState, uint8_t iButton);

/**
 * @brief 마우스 버튼 다운 SDL_Event 를 생성한다.
 */
void CreateMouseButtonDownEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                              uint8_t iButton);

/**
 * @brief 마우스 버튼 업 SDL_Event 를 생성한다.
 */
void CreateMouseButtonUpEvt(SDL_Event *pEvt, int32_t iX, int32_t iY,
                            uint8_t iButton);

/**
 * @brief 지정 좌표에 마우스 클릭 이벤트를 주입한다.
 *
 * TEST_VISUAL=1 환경변수가 설정된 경우:
 *   SDL_WarpMouseGlobal 로 실제 커서를 이동시킨 뒤 ButtonDown/Up 만 ExtEvt_T 에 주입.
 *   (ImGui 가 GlobalMouseState 로 커서 위치를 읽으므로 Warp 만으로 hover 판정 가능)
 *
 * 그 외 (offscreen 등):
 *   Motion/Down/Up 3개 이벤트를 모두 ExtEvt_T 에 주입.
 *
 * edge 회피를 위해 X_OFFSET(5), Y_OFFSET(5) 가 더해진다.
 *
 * @param[in,out] pECS    ECS 레지스트리 (ctx 에 ExtEvt_T 가 있어야 한다)
 * @param[in]     iX      클릭 위젯 X 좌표 (윈도우 클라이언트 좌표)
 * @param[in]     iY      클릭 위젯 Y 좌표 (윈도우 클라이언트 좌표)
 * @param[in]     iButton SDL_BUTTON_LEFT 등
 */
void Click_Evt(std::shared_ptr<entt::registry> &pECS,
               int32_t iX, int32_t iY, uint8_t iButton);

/**
 * @brief WidgetRegistry_T 에서 위젯 이름으로 좌표를 찾아 클릭 이벤트를 생성한다.
 *
 * @param[in,out] pECS      ECS 레지스트리
 * @param[in]     funcName  UI 함수 이름 (예: "UI_MainMenu")
 * @param[in]     itemName  위젯 이름 (예: "Start")
 * @param[in]     iButton   SDL_BUTTON_LEFT 등
 * @return true 위젯을 찾아 이벤트를 등록한 경우, false 찾지 못한 경우
 */
bool Click_Evt(std::shared_ptr<entt::registry> &pECS,
               const std::string &funcName, const std::string &itemName,
               uint8_t iButton);

