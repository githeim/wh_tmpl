#pragma once
#include "SDL2_Ctx.h"
/**
 * @brief ImGui 컨텍스트와 SDL2/SDLRenderer 백엔드를 초기화한다.
 *
 * @param[in] sdlCtx SDL 윈도우/렌더러를 포함한 컨텍스트
 * @return 성공 시 true, 실패 시 false
 */
bool InitImGui(SDL2Ctx &sdlCtx); 

/**
 * @brief ImGui 백엔드와 컨텍스트를 안전하게 종료한다.
 */
void CleanupImGui();
