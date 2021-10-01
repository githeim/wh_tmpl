#include <stdio.h>
#include "libmodule.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <string>

#include <chrono>

#include <unistd.h>


// for box2d
#include "box2d/box2d.h"

#include "Physic_world.h"

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
 * @param dbTimeDiff[OUT]
 *
 * @return  Actual FPS
 */
double Frame_Rate_Control(double dbFPS,double &dbTimeDiff_SEC){
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
  dbTimeDiff_SEC = Actual_Frame_diff_SEC.count();
  return (1.0f/dbTimeDiff_SEC);
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
  CreateWorld();

  pImgTexture = IMG_LoadTexture(pRenderer, "resource/pics/gui_common_ex00.png");
  if (!pImgTexture)
    printf("\033[1;33m[%s][%d] :x: error %s \033[m\n",__FUNCTION__,__LINE__,SDL_GetError());

  int iWidth,iHeight;
  SDL_QueryTexture(pImgTexture, NULL, NULL, &iWidth, &iHeight);


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
  double dbActualFPS=0.0f;
  //ground size (40M,20M)
  SDL_Rect rectGround={
    SCREEN_WIDTH /2 + (int)(  0.0f*10) - (int)(40.0f/2)*10,
    SCREEN_HEIGHT/2 - (int)(-10.0f*10) - (int)(20.0f/2)*10,
    40*10,20*10};

  // box size (2M,2M)
  SDL_Rect rectBox={
    SCREEN_WIDTH /2 + (int)(  0.0f)*10 - (int)(2.0f/2)*10,
    SCREEN_HEIGHT/2 - (int)( 30.0f)*10 - (int)(2.0f/2)*10,
    20,20};
  SDL_Surface* pSurfRect = SDL_CreateRGBSurface(0, 20, 20,32,0,0,0,0 );
  SDL_FillRect(pSurfRect,NULL,SDL_MapRGB(pSurfRect->format,255,0,0));
  SDL_Texture *pTexBox00 = SDL_CreateTextureFromSurface(pRenderer,pSurfRect);
  SDL_FreeSurface(pSurfRect);

  double dbTimeDiff;
  float fBox_X_M          = 0.0f;
  float fBox_Y_M          = 4.0f;
  float fBox_Angle_DEGREE = .0f;
  while (!bQuit) {

    if (bQuit ==true) 
      break;
    // clear screen
    SDL_SetRenderDrawColor(pRenderer, 55, 55, 55, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(pRenderer);
    

    SDL_SetRenderDrawColor(pRenderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawRect(pRenderer, &rectGround);
    
    SDL_SetRenderDrawColor(pRenderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    rectBox.x = SCREEN_WIDTH / 2  +(int)(fBox_X_M*10.0f)- (int)(2.0f/2)*10;
    rectBox.y = SCREEN_HEIGHT/ 2  -(int)(fBox_Y_M*10.0f)- (int)(2.0f/2)*10;
    SDL_RenderCopyEx(pRenderer, pTexBox00, NULL,&rectBox, fBox_Angle_DEGREE, NULL, SDL_FLIP_NONE);

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
            ForceToLeft();
            break;
          case SDLK_d:
            ForceToRight();
            break;
          case SDLK_w:
            ForceToUp();
            break;
          case SDLK_s:
            ForceToDown();
            break;
        }
      }

    }
    // FPS control
    dbActualFPS = Frame_Rate_Control(SCREEN_FPS,dbTimeDiff);
    SpinWorld (dbTimeDiff,
        fBox_X_M,
        fBox_Y_M,
        fBox_Angle_DEGREE);

  }
  DestroyWorld();
  SDL_Delay( 100 );

  SDL_DestroyTexture(pTxtTexture);
  SDL_DestroyTexture(pImgTexture);
  SDL_DestroyTexture(pTexBox00);
  DeInit_Fonts(pFont);
  DeInit_SDL_ctx(pWindow, pRenderer);

  return 0;
}
