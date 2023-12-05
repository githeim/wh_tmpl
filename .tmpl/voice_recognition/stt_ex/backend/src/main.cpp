#include <stdio.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <string>

#include <chrono>

#include <unistd.h>
#include <vector>
#include <mutex>
#include "MsgReceiver.h"
#include "MsgHandler.h"

#define SCREEN_POS_X (50)
#define SCREEN_POS_Y (50)

#define SCREEN_WIDTH  (800)
#define SCREEN_HEIGHT (40)

#define SCREEN_FPS (20)
#define POINTS_COUNT (4)

#define DEFAULT_OPACITY (.6f)
#define DEFAULT_FONT_SIZE (28)

int Init_SDL_ctx(SDL_Window* &pWindow, SDL_Renderer* &pRenderer) {
  pWindow = NULL;

  //Initialize SDL
  if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
  {
    printf("\033[1;31m[%s][%d] :x: Could not Init SDL2 [%s] \033[m\n",
        __FUNCTION__,__LINE__, SDL_GetError());
    return -1;
  }

  pWindow = SDL_CreateWindow( "Simple_STT_Example", 
      SCREEN_POS_X, SCREEN_POS_Y, 
      SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN|SDL_WINDOW_BORDERLESS );
  if( pWindow == NULL )
  {
    printf("\033[1;31m[%s][%d] :x: error %s \033[m\n",
        __FUNCTION__,__LINE__,SDL_GetError());
    return -1;
  }

	pRenderer = SDL_CreateRenderer( pWindow, -1, 
      SDL_RENDERER_ACCELERATED  );

  if (!pRenderer) {
    printf("\033[1;31m[%s][%d] :x: error %s \033[m\n",
        __FUNCTION__,__LINE__,SDL_GetError());
    return -1;
  }

  return 0;
}

int DeInit_SDL_ctx(SDL_Window* &pWindow, SDL_Renderer* &pRenderer) {
  SDL_RenderClear(pRenderer);
  SDL_DestroyRenderer(pRenderer);
  //Destroy window
  SDL_DestroyWindow( pWindow );
  pWindow = NULL;
  //Quit SDL subsystems
  SDL_Quit();

  return 0;
}

int Init_Fonts(TTF_Font* &pFont) {
  if( TTF_Init() == -1 ) {
    printf("\033[1;33m[%s][%d] :x: SDL_ttf could not initialize!"
        " SDL_ttf Error: %s \033[m\n",__FUNCTION__,__LINE__,TTF_GetError());
    return -1;
  }
	pFont = TTF_OpenFont( "resource/fonts/NanumGothicCoding-Regular.ttf", DEFAULT_FONT_SIZE );
	if( pFont == NULL )
	{
    printf("\033[1;31m[%s][%d] :x: Failed to load font [%s] \033[m\n",
        __FUNCTION__,__LINE__,TTF_GetError());
    return -1;
	}
  else {
    
  }
  return 0;
}
int DeInit_Fonts(TTF_Font* &pFont) {
	TTF_CloseFont( pFont );
  return 0;
}

/**
 * @brief Create Texture of Text. warning! pTxtTexture should be destroy after
 *        using it.(ex :  SDL_DestroyTexture(pTxtTexture);) it causes memory leak
 *
 * @param pTxtTexture[OUT]
 * @param pFont[IN] font instance to use
 * @param strText[IN] text to draw
 * @param pRenderer[IN] renderer to use
 *
 * @return 
 */
int DrawText(SDL_Texture* &pTxtTexture,TTF_Font* &pFont,
    std::string strText,SDL_Renderer* &pRenderer) 
{
  SDL_Color textColor = { 255, 255, 0 };
	SDL_Surface* pTxtSurface = TTF_RenderText_Solid( pFont, strText.c_str(), 
      textColor );
  if( pTxtSurface == NULL )
	{
    printf("\033[1;31m[%s][%d] :x: SDL_ttf Error: %s \033[m\n",
        __FUNCTION__,__LINE__, TTF_GetError());
    return -1;
	}
  pTxtTexture = SDL_CreateTextureFromSurface(pRenderer, pTxtSurface);
  SDL_FreeSurface(pTxtSurface);
	
  return 0;
}

/**
 * @brief Control frame rates
 *
 * @param dbFPS[IN]
 *
 * @return  Actual FPS
 */
double Frame_Rate_Control(double dbFPS){
  static auto Frame_start = std::chrono::system_clock::now();
  static auto Frame_end   = std::chrono::system_clock::now();
  std::chrono::duration<double> Frame_diff_SEC;
  std::chrono::duration<double> Actual_Frame_diff_SEC;

  Frame_end = std::chrono::system_clock::now();
  Frame_diff_SEC = Frame_end - Frame_start;
  // :x:  Reference frame interval
  double dbRef_SEC= 1.0f/dbFPS;

  if (Frame_diff_SEC.count() < dbRef_SEC) {
    usleep((dbRef_SEC - Frame_diff_SEC.count())*1000000);
  }

  Frame_start = std::chrono::system_clock::now();
  Actual_Frame_diff_SEC = Frame_start - Frame_end;
  return (1.0f/Actual_Frame_diff_SEC.count());
}

int main(int argc, char *argv[]) {
  std::mutex mtxSTT_Msg;
  std::vector<std::string> vecSTT_Msg;
  
  // Start posix MQ receiver
  StartMsgReceiver(vecSTT_Msg,mtxSTT_Msg);

  SDL_Window* pWindow = NULL;
  SDL_Renderer* pRenderer = NULL;

  TTF_Font *pFont = NULL;
  SDL_Texture* pTxtTexture;

  if ( Init_SDL_ctx(pWindow,pRenderer) )
    return -1;

  if ( Init_Fonts(pFont) ) {
    return -1;
  }

  SDL_Rect Txt_Pos={10,10,0,0}; 

  // Draw Text
  if (
      DrawText(pTxtTexture,pFont,
        std::string("Q : exit, w/s : change opacity"),pRenderer)
     ) {
    printf("\033[1;31m[%s][%d] :x: Draw Text error \033[m\n",
        __FUNCTION__,__LINE__);
    return -1;
  }
  SDL_QueryTexture(pTxtTexture, NULL, NULL, &Txt_Pos.w, &Txt_Pos.h);
  SDL_DestroyTexture(pTxtTexture);

  SDL_Event Evt;
  bool bQuit = false;
  float fOpaque=DEFAULT_OPACITY;
  //SDL_SetWindowAlwaysOnTop(pWindow,SDL_TRUE);


  std::string strSTT_Msg="";

  while (!bQuit) {
    SDL_SetWindowOpacity(pWindow, fOpaque);
    if (bQuit ==true) 
      break;
    // clear screen
    SDL_SetRenderDrawColor(pRenderer, 0,0,0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(pRenderer);

    SDL_SetRenderDrawColor(pRenderer, 0, 0,0, SDL_ALPHA_OPAQUE);
    // draw text

    mtxSTT_Msg.lock();
    if (vecSTT_Msg.size() !=0) {
      strSTT_Msg = vecSTT_Msg[0];
      vecSTT_Msg.erase(vecSTT_Msg.begin());
      printf("\033[1;33m[%s][%d] :x: size [%ld] txt [%s]  \033[m\n",
          __FUNCTION__,__LINE__,vecSTT_Msg.size(),strSTT_Msg.c_str());
      Handle_STT_Msg(strSTT_Msg);
    }
    mtxSTT_Msg.unlock();
    DrawText(pTxtTexture,pFont,
        std::string("q : exit, w/s : change opacity , opacity =") +
        std::to_string(fOpaque)+std::string(" ")+strSTT_Msg,pRenderer);


    SDL_RenderCopy(pRenderer, pTxtTexture, NULL, &Txt_Pos );
    SDL_DestroyTexture(pTxtTexture);
    // render
    SDL_RenderPresent(pRenderer);

    // evt handling
    while( SDL_PollEvent( &Evt ) != 0 )
    {
      //User requests quit
      if( Evt.type == SDL_QUIT )
      {
        bQuit = true;
      }
      else if( Evt.type == SDL_KEYDOWN )
      {
        switch( Evt.key.keysym.sym )
        {
          case SDLK_q:
            bQuit = true;
            break;
          case SDLK_w:
            if (fOpaque < 0.95)
              fOpaque+=0.05;
            break;
          case SDLK_s:
            if (fOpaque > 0.05)
              fOpaque-=0.05;
            break;
        }
      }
    }
    // FPS control
    Frame_Rate_Control(SCREEN_FPS);
  }
  printf("\033[1;33m[%s][%d] :x: Quit \033[m\n",__FUNCTION__,__LINE__);


  SDL_Delay( 100 );
  SDL_DestroyTexture(pTxtTexture);
  DeInit_Fonts(pFont);
  DeInit_SDL_ctx(pWindow, pRenderer);

  // Stop MQ Msg Receiver
  StopMsgReceiver(); 
  return 0;
}
