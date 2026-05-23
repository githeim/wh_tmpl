#include "Log_Util.h"
#include "SDL2_Ctx.h"

namespace {
std::map<std::string, std::string> g_mapSoundPath {
  { "tang", "resource/sound/tang.wav" },
  { "boom", "resource/sound/boom.wav" }
};
}

int Init_Joystick(AppCtx &Ctx);
void DeInit_Joystick(AppCtx &Ctx);
int Init_Sound(AppCtx &Ctx);
void DeInit_Sound(AppCtx &Ctx);

/**
 * @brief SDL2 컨텍스트를 초기화한다.
 *
 * SDL2 비디오/조이스틱/오디오 서브시스템을 초기화하고,
 * 윈도우 및 렌더러를 생성한다. 조이스틱과 사운드도 함께 초기화한다.
 *
 * @param[out] Ctx 초기화할 AppCtx 구조체 참조
 * @return 성공 시 0, 실패 시 -1
 */
int Init_SDL_ctx(AppCtx &Ctx) {
  Ctx.pWindow = nullptr;
  Ctx.pRenderer = nullptr;

  //Initialize SDL
  if( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_AUDIO ) <0)
  {
    LER("Could not Init SDL2 [" << SDL_GetError() << "]");
    return -1;
  }

  Ctx.pWindow = SDL_CreateWindow( "app_base",
      SCREEN_POS_X, SCREEN_POS_Y,
      SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
  if( Ctx.pWindow == NULL )
  {
    LER("error " << SDL_GetError());
    return -1;
  }
  // initial cursor position
  SDL_WarpMouseInWindow(Ctx.pWindow, INITIAL_CURSOR_POS_X, INITIAL_CURSOR_POS_Y);
  Ctx.pRenderer = SDL_CreateRenderer( Ctx.pWindow, -1,
      SDL_RENDERER_ACCELERATED  );

  if (!Ctx.pRenderer) {
    LER("error " << SDL_GetError());
    return -1;
  }

  if (Init_Joystick(Ctx)) {
    return -1;
  }
  if (Init_Sound(Ctx)) {
    LWY("Warning: Sound init failed, continue without audio");
  }
  if (Init_Fonts(Ctx)) {
    return -1;
  }

  return 0;
}

/**
 * @brief SDL2 컨텍스트를 정리(해제)한다.
 *
 * 폰트, 사운드, 조이스틱, 렌더러, 윈도우를 순서대로 해제하고
 * SDL2 서브시스템을 종료한다.
 *
 * @param[in,out] Ctx 정리할 AppCtx 구조체 참조
 * @return 항상 0 반환
 */
int DeInit_SDL_ctx(AppCtx &Ctx) {
  DeInit_Fonts(Ctx);
  DeInit_Sound(Ctx);
  DeInit_Joystick(Ctx);
  if (Ctx.pRenderer != nullptr) {
    SDL_RenderClear(Ctx.pRenderer);
    SDL_DestroyRenderer(Ctx.pRenderer);
    Ctx.pRenderer = nullptr;
  }
  //Destroy window
  if (Ctx.pWindow != nullptr) {
    SDL_DestroyWindow( Ctx.pWindow );
    Ctx.pWindow = nullptr;
  }
  //Quit SDL subsystems
  SDL_Quit();

  return 0;
}

/**
 * @brief SDL_ttf를 초기화하고 기본 폰트를 로드한다.
 *
 * NanumGothicCoding-Regular.ttf 폰트를 지정한 크기로 로드하여
 * Ctx.pFontMain에 설정한다.
 *
 * @param[out] Ctx 폰트를 설정할 AppCtx 구조체 참조
 * @param[in]  iFontSize 로드할 폰트 크기 (기본값: DEFAULT_FONT_SIZE)
 * @return 성공 시 0, 실패 시 -1
 */
int Init_Fonts(AppCtx &Ctx, int iFontSize) {
  if( TTF_Init() == -1 ) {
    LWY("SDL_ttf could not initialize! SDL_ttf Error: " << TTF_GetError());
    return -1;
  }

  Ctx.pFontMain = TTF_OpenFont( "resource/fonts/NanumGothicCoding-Regular.ttf",
                                iFontSize );
  if( Ctx.pFontMain == NULL )
  {
    LER("Failed to load font [" << TTF_GetError() << "]");
    return -1;
  }
  return 0;
}

/**
 * @brief 폰트 리소스를 해제하고 SDL_ttf를 종료한다.
 *
 * Ctx.pFontMain이 유효한 경우 폰트를 닫고 nullptr로 초기화한다.
 * 이후 TTF_Quit()을 호출하여 SDL_ttf 서브시스템을 종료한다.
 *
 * @param[in,out] Ctx 폰트를 해제할 AppCtx 구조체 참조
 * @return 항상 0 반환
 */
int DeInit_Fonts(AppCtx &Ctx) {
  if (Ctx.pFontMain != nullptr) {
    TTF_CloseFont(Ctx.pFontMain);
    Ctx.pFontMain = nullptr;
  }
  TTF_Quit();
  return 0;
}

/**
 * @brief 문자열을 SDL_Texture로 렌더링한다.
 *
 * 지정한 폰트와 텍스트로 SDL_Surface를 생성한 뒤 SDL_Texture로 변환한다.
 * 텍스트 색상은 노란색(255, 255, 0)으로 고정된다.
 * 생성된 Surface는 즉시 해제되며, Texture는 호출자가 관리해야 한다.
 *
 * @param[out] pTxtTexture 생성된 텍스트 텍스처 포인터 참조
 * @param[in]  pFont       렌더링에 사용할 TTF_Font 포인터 참조
 * @param[in]  strText     렌더링할 문자열
 * @param[in]  pRenderer   텍스처 생성에 사용할 SDL_Renderer 포인터 참조
 * @return 성공 시 0, 실패 시 -1
 */
int DrawText(SDL_Texture* &pTxtTexture, TTF_Font* &pFont,
    std::string strText, SDL_Renderer* &pRenderer)
{
  SDL_Color textColor = { 255, 255, 0 };
  SDL_Surface* pTxtSurface = TTF_RenderText_Solid( pFont, strText.c_str(),
      textColor );
  if( pTxtSurface == NULL )
  {
    LER("SDL_ttf Error: " << TTF_GetError());
    return -1;
  }
  pTxtTexture = SDL_CreateTextureFromSurface(pRenderer, pTxtSurface);
  SDL_FreeSurface(pTxtSurface);

  return 0;
}

/**
 * @brief 연결된 조이스틱을 열고 컨텍스트에 등록한다.
 *
 * 연결된 조이스틱의 수를 확인하고, 각 조이스틱을 열어
 * Ctx.mapJoystick에 저장한다. 조이스틱이 없어도 오류를 반환하지 않는다.
 *
 * @param[out] Ctx 조이스틱을 등록할 AppCtx 구조체 참조
 * @return 항상 0 반환
 */
int Init_Joystick(AppCtx &Ctx) {
  LIY("Init Joystick");
  int iNumJoysticks = SDL_NumJoysticks();

  if( iNumJoysticks < 1 )
  {
    LWR("Warning: No joysticks connected!");
  }
  else
  {
    LIY("The number of joysticks = " << iNumJoysticks);

    SDL_JoystickEventState(SDL_ENABLE);
    for (int i =0 ; i < iNumJoysticks ; i++ ) {
      SDL_Joystick *pJoystick = SDL_JoystickOpen(i);
      LIY("Open Joystick [" << i << "]");

      if (pJoystick != nullptr) {
        Ctx.mapJoystick[i] = pJoystick;
      } else {
        LER("Error Joystick [" << i << "] Unable to open");
      }
    }
  }
  LIY("Init Joystick Done");
  return 0;
}

/**
 * @brief 등록된 모든 조이스틱을 닫고 해제한다.
 *
 * Ctx.mapJoystick에 저장된 모든 조이스틱 핸들을 닫고,
 * 맵을 비운다.
 *
 * @param[in,out] Ctx 조이스틱을 해제할 AppCtx 구조체 참조
 */
void DeInit_Joystick(AppCtx &Ctx) {
  for (auto &Item : Ctx.mapJoystick) {
    SDL_Joystick* &pJoystick = Item.second;
    if (pJoystick != nullptr) {
      SDL_JoystickClose(pJoystick);
      pJoystick = nullptr;
    }
  }
  Ctx.mapJoystick.clear();
  LIY("DeInit Joystick Done");
}

/**
 * @brief SDL_mixer를 초기화하고 사운드 파일을 로드한다.
 *
 * 44100Hz, 스테레오 설정으로 오디오 장치를 열고,
 * g_mapSoundPath에 정의된 WAV 파일들을 로드하여 Ctx.mapSound에 저장한다.
 *
 * @param[out] Ctx 사운드를 로드할 AppCtx 구조체 참조
 * @return 성공 시 0, 오디오 장치 오픈 실패 시 -1
 */
int Init_Sound(AppCtx &Ctx) {
  LIY("Init Sound");
  if( Mix_OpenAudio( 44100, MIX_DEFAULT_FORMAT, 2, 2048 ) < 0 )
  {
    LER("SDL_mixer , Init Err");
    return -1;
  }

  LIY("SDL mixer start");
  for (const auto &item : g_mapSoundPath) {
    Mix_Chunk *pSound;
    const std::string &strName = item.first;
    const std::string &strFilePath = item.second;
    pSound = Mix_LoadWAV(strFilePath.c_str());
    if (pSound) {
      Ctx.mapSound[strName] = pSound;
    }
  }

  return 0;
}

/**
 * @brief 로드된 모든 사운드를 해제하고 SDL_mixer를 종료한다.
 *
 * Ctx.mapSound에 저장된 모든 Mix_Chunk를 해제하고 맵을 비운다.
 * 이후 Mix_CloseAudio()를 호출하여 오디오 장치를 닫는다.
 *
 * @param[in,out] Ctx 사운드를 해제할 AppCtx 구조체 참조
 */
void DeInit_Sound(AppCtx &Ctx) {
  for (auto &item : Ctx.mapSound) {
    Mix_Chunk *&pSound = item.second;
    if (pSound) {
      Mix_FreeChunk(pSound);
      pSound = nullptr;
      LIY("DeInit sound [" << item.first << "]");
    }
  }
  Ctx.mapSound.clear();
  Mix_CloseAudio();
  LIY("DeInit Sound Done");
}

/**
 * @brief 조이스틱 상태를 주기적으로 확인한다. (현재 미구현)
 *
 * 게임 루프에서 주기적으로 호출되어 조이스틱 입력을 처리할 목적으로
 * 선언된 함수이나, 현재는 빈 구현체다.
 *
 * @param[in] Ctx       AppCtx 구조체 참조 (현재 미사용)
 * @param[in] dbTimeDiff 이전 프레임과의 경과 시간(초) (현재 미사용)
 */
void Check_Joystick(AppCtx &Ctx, double dbTimeDiff) {
  (void)Ctx;
  (void)dbTimeDiff;
}

/**
 * @brief 사운드 재생 여부를 설정한다.
 *
 * @param[out] Ctx    AppCtx 구조체 참조
 * @param[in]  bOnOff true이면 사운드 활성화, false이면 비활성화
 */
void Set_Sound(AppCtx &Ctx, bool bOnOff) {
  Ctx.bSoundOnOff = bOnOff;
}

/**
 * @brief 지정한 이름의 사운드를 재생한다.
 *
 * 사운드가 활성화(bSoundOnOff == true)된 상태이고,
 * strSound에 해당하는 사운드가 Ctx.mapSound에 존재할 경우 재생한다.
 * 빈 채널(-1)을 사용하여 첫 번째 사용 가능한 채널에서 1회 재생한다.
 *
 * @param[in] Ctx      AppCtx 구조체 참조
 * @param[in] strSound 재생할 사운드 이름 (mapSound의 키)
 */
void Play_Sound(AppCtx &Ctx, const std::string &strSound) {
  if (Ctx.bSoundOnOff && Ctx.mapSound.find(strSound) != Ctx.mapSound.end()) {
    Mix_PlayChannel(-1, Ctx.mapSound[strSound], 0);
  }
}
