#include <stdio.h>
#include "libmodule.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <string>

#include <chrono>

#include <unistd.h>

#define SCREEN_POS_X (50)
#define SCREEN_POS_Y (50)

#define SCREEN_WIDTH  (640)
#define SCREEN_HEIGHT (480)

#define SCREEN_FPS (50)
#define POINTS_COUNT (4)

int Init_SDL_ctx(SDL_Window* &pWindow, SDL_Renderer* &pRenderer) {
  pWindow = NULL;

  //Initialize SDL
  if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
  {
    printf("\033[1;31m[%s][%d] :x: Could not Init SDL2 [%s] \033[m\n",
        __FUNCTION__,__LINE__, SDL_GetError());
    return -1;
  }

  pWindow = SDL_CreateWindow( "WW_ProjectName_WW", 
      SCREEN_POS_X, SCREEN_POS_Y, 
      SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
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
	pFont = TTF_OpenFont( "resource/fonts/NanumGothicCoding-Regular.ttf", 28 );
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
  printf("\033[1;33m[%s][%d] :x: %f seconds elapsed %f frames ref %f\033[m\n",
      __FUNCTION__,__LINE__,Frame_diff_SEC.count() , 1.0f/Frame_diff_SEC.count(),
      dbRef_SEC);
  if (Frame_diff_SEC.count() < dbRef_SEC) {
    usleep((dbRef_SEC - Frame_diff_SEC.count())*1000000);
  }

  Frame_start = std::chrono::system_clock::now();
  Actual_Frame_diff_SEC = Frame_start - Frame_end;
  return (1.0f/Actual_Frame_diff_SEC.count());
}

int main(int argc, char *argv[]) {
  printf("Project New_Project\n");

  SDL_Window* pWindow = NULL;
  SDL_Renderer* pRenderer = NULL;

  TTF_Font *pFont = NULL;
  SDL_Texture* pImgTexture;
  SDL_Texture* pTxtTexture;


  if ( Init_SDL_ctx(pWindow,pRenderer) )
    return -1;

  if ( Init_Fonts(pFont) ) {
    return -1;
  }

  pImgTexture = IMG_LoadTexture(pRenderer, "resource/pics/gui_common_ex00.png");
  if (!pImgTexture)
    printf("\033[1;33m[%s][%d] :x: error %s \033[m\n",__FUNCTION__,__LINE__,SDL_GetError());

  SDL_Rect posTexture00; 
  SDL_Rect posTexture01; 
  int iWidth,iHeight;
  SDL_QueryTexture(pImgTexture, NULL, NULL, &iWidth, &iHeight);
  // Keep Position Center
  posTexture00.x = SCREEN_WIDTH/2  - (iWidth/2); 
  posTexture00.y = SCREEN_HEIGHT/2 - (iHeight/2); 
  posTexture00.w = iWidth; 
  posTexture00.h = iHeight;

  posTexture01.x = 30; 
  posTexture01.y = 30; 
  posTexture01.w = 30; 
  posTexture01.h = 30; 

  SDL_Rect Txt_Pos={10,10,0,0}; 



  // Draw Text
  if (
      DrawText(pTxtTexture,pFont,
        std::string("Text Test: Press Q , to exit"),pRenderer)
     ) {
    printf("\033[1;31m[%s][%d] :x: Draw Text error \033[m\n",
        __FUNCTION__,__LINE__);
    return -1;
  }
  SDL_QueryTexture(pTxtTexture, NULL, NULL, &Txt_Pos.w, &Txt_Pos.h);

  SDL_Event Evt;
  bool bQuit = false;
  double dbAngle =0.0f;
  double dbActualFPS=0.0f;

  while (!bQuit) {

    if (bQuit ==true) 
      break;
    // clear screen
    SDL_SetRenderDrawColor(pRenderer, 55, 55, 55, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(pRenderer);
    // draw texture
    SDL_RenderCopyEx(pRenderer, pImgTexture, NULL, &posTexture00,dbAngle, NULL, SDL_FLIP_NONE );
    SDL_RenderCopyEx(pRenderer, pImgTexture, NULL, &posTexture01,dbAngle, NULL, SDL_FLIP_NONE );

    // draw lines (draw triangle in the center)
    static SDL_Point points[POINTS_COUNT] = {
      {320, 200},
      {300, 240},
      {340, 240},
      {320, 200}
    };
    SDL_SetRenderDrawColor(pRenderer, 255, 55, 55, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLines(pRenderer, points, POINTS_COUNT);
    SDL_SetRenderDrawColor(pRenderer, 55, 55, 55, SDL_ALPHA_OPAQUE);
    // draw text
    DrawText(pTxtTexture,pFont,
        std::string("Text Test: Press Q , to exit,FPS =") +std::to_string(dbActualFPS),pRenderer);
    SDL_RenderCopy(pRenderer, pTxtTexture, NULL, &Txt_Pos );

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

          case SDLK_a:
            dbAngle-=1;
            break;
          case SDLK_d:
            dbAngle+=1;
            break;
          case SDLK_w:
            break;
          case SDLK_s:
            break;
        }
      }

    }
    // FPS control
    dbActualFPS = Frame_Rate_Control(SCREEN_FPS);
  }

  SDL_Delay( 100 );

  SDL_DestroyTexture(pTxtTexture);
  SDL_DestroyTexture(pImgTexture);
  DeInit_Fonts(pFont);
  DeInit_SDL_ctx(pWindow, pRenderer);

  return 0;
}
