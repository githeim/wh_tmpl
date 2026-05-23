#ifndef _SDL2_CTX_H_
#define _SDL2_CTX_H_
#include <chrono>
#include <string>
#include <unistd.h>
#include <map>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_mixer.h>

#define SCREEN_POS_X (0)    ///< 윈도우 초기 X 위치 (픽셀)
#define SCREEN_POS_Y (0)    ///< 윈도우 초기 Y 위치 (픽셀)

#define SCREEN_WIDTH  (640) ///< 기본 윈도우 너비 (픽셀)
#define SCREEN_HEIGHT (480) ///< 기본 윈도우 높이 (픽셀)

#define SCREEN_FPS (50)                         ///< 목표 FPS

#define INITIAL_CURSOR_POS_X (100) ///< 시작 시 마우스 커서 X 좌표
#define INITIAL_CURSOR_POS_Y (100) ///< 시작 시 마우스 커서 Y 좌표

#define DEFAULT_ALPHA (SDL_ALPHA_OPAQUE)      ///< 기본 알파값 (불투명)
#define COLOR_RED        {255,0  ,0  , DEFAULT_ALPHA} ///< 빨간색 RGBA
#define COLOR_GREEN      {0  ,255,0  , DEFAULT_ALPHA} ///< 초록색 RGBA
#define COLOR_BLUE       {0  ,0  ,255, DEFAULT_ALPHA} ///< 파란색 RGBA
#define COLOR_YELLOW     {255,255,0  , DEFAULT_ALPHA} ///< 노란색 RGBA
#define COLOR_DARK_GRAY  {169,169,169, DEFAULT_ALPHA} ///< 진한 회색 RGBA
#define COLOR_DEFAULT COLOR_GREEN                     ///< 기본 색상 (초록)

#define DEFAULT_FONT_SIZE (18) ///< 기본 폰트 크기 (pt)

/**
 * @brief SDL2 애플리케이션 컨텍스트 구조체.
 *
 * 윈도우, 렌더러, 폰트, 조이스틱, 사운드 등
 * 애플리케이션 전반에서 공유되는 SDL2 리소스를 묶어 관리한다.
 */
struct AppCtx {
  SDL_Window*   pWindow   = nullptr; ///< SDL2 윈도우 핸들
  SDL_Renderer* pRenderer = nullptr; ///< SDL2 하드웨어 가속 렌더러 핸들
  TTF_Font*     pFontMain = nullptr; ///< 기본 TTF 폰트 핸들 (NanumGothicCoding)

  std::map<int, SDL_Joystick*>       mapJoystick; ///< 조이스틱 인덱스 -> SDL_Joystick* 매핑
  std::map<std::string, Mix_Chunk*>  mapSound;    ///< 사운드 이름 -> Mix_Chunk* 매핑
  bool bSoundOnOff = false;                       ///< 사운드 재생 활성화 여부 (true: 재생, false: 무음)
};

/**
 * @brief SDL 윈도우/렌더러/입력/오디오/폰트를 초기화한다.
 *
 * @param[out] Ctx SDL 리소스를 저장할 애플리케이션 컨텍스트
 * @return 성공 시 0, 실패 시 -1
 */
int Init_SDL_ctx(AppCtx &Ctx);
/**
 * @brief SDL 관련 리소스를 해제하고 서브시스템을 종료한다.
 *
 * @param[in,out] Ctx 정리할 애플리케이션 컨텍스트
 * @return 항상 0
 */
int DeInit_SDL_ctx(AppCtx &Ctx);

/**
 * @brief 기본 폰트를 로드한다.
 *
 * @param[out] Ctx 폰트 리소스를 보관할 애플리케이션 컨텍스트
 * @param[in]  iFontSize 로드할 폰트 크기
 * @return 성공 시 0, 실패 시 -1
 */
int Init_Fonts(AppCtx &Ctx, int iFontSize = DEFAULT_FONT_SIZE);
/**
 * @brief 로드된 폰트를 해제한다.
 *
 * @param[in,out] Ctx 폰트 리소스를 포함한 애플리케이션 컨텍스트
 * @return 항상 0
 */
int DeInit_Fonts(AppCtx &Ctx);
/**
 * @brief 조이스틱 상태를 점검한다.
 *
 * @param[in] Ctx SDL 애플리케이션 컨텍스트
 * @param[in] dbTimeDiff 직전 프레임 대비 경과 시간(초)
 */
void Check_Joystick(AppCtx &Ctx, double dbTimeDiff);
/**
 * @brief 사운드 재생 활성화 여부를 설정한다.
 *
 * @param[in,out] Ctx SDL 애플리케이션 컨텍스트
 * @param[in]     bOnOff true 이면 사운드 사용, false 이면 비사용
 */
void Set_Sound(AppCtx &Ctx, bool bOnOff);
/**
 * @brief 이름으로 등록된 사운드를 재생한다.
 *
 * @param[in] Ctx SDL 애플리케이션 컨텍스트
 * @param[in] strSound 재생할 사운드 키 이름
 */
void Play_Sound(AppCtx &Ctx, const std::string &strSound);
#endif /* ifndef _SDL2_CTX_H_ */
