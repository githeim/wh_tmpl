#include <stdio.h>
#include "libmodule.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer.h"

#include "SDL.h"
#include <unistd.h>

int main(int argc, char *argv[]) {

  // Setup SDL
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
  {
      printf("Error: %s\n", SDL_GetError());
      return -1;
  }

  // Create window with SDL_Renderer graphics context
  SDL_WindowFlags window_flags = 
             (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  SDL_Window* window = SDL_CreateWindow("imgui + SDL2 example", 
                                        SDL_WINDOWPOS_CENTERED, 
                                        SDL_WINDOWPOS_CENTERED, 
                                        640, 480, window_flags);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 
                          SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  if (renderer == NULL)
  {
    SDL_Log("Error creating SDL_Renderer!");
    return 0;
  }


  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  // Enable Keyboard Controls
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     
  // Enable Gamepad Controls
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  //ImGui::StyleColorsLight();
  //ImGui::StyleColorsClassic();


  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer_Init(renderer);

  // Check box variables
  bool show_demo_window = false;
  bool show_another_window = false;

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // Main loop
  bool done = false;
  while (!done)
  {
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        done = true;
      if (event.type == SDL_WINDOWEVENT && 
          event.window.event == SDL_WINDOWEVENT_CLOSE && 
          event.window.windowID == SDL_GetWindowID(window)
          )
        done = true;
    }

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    // if there is NewFrame() function call, ImGui::Render() should be called
    ImGui::NewFrame();
    // Create Window
    {
      static float f = 0.0f;
      static int counter = 0;

      bool bOpen;
      ImGui::Begin("Test window");
      // without window frame
      //ImGui::Begin("Test window",&bOpen,ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoBackground);
      
      // Display some text (you can use a format strings too)
      ImGui::Text("This is some useful text.");               
      ImGui::Checkbox("Demo Window", &show_demo_window);      
      ImGui::Checkbox("Another Window", &show_another_window);

      ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            
      ImGui::ColorEdit3("clear color", (float*)&clear_color); 

      if (ImGui::Button("Button"))                            
        counter++;
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }
    if (show_another_window)
    {
      ImGui::Begin("Another Window", &show_another_window);   
      ImGui::Text("Hello from another window!");
      if (ImGui::Button("Close Me"))
        show_another_window = false;
      ImGui::End();
    }
    if (show_demo_window)
      ImGui::ShowDemoWindow(&show_demo_window);

    // The Test window cannot move because of this API(SetWindowPos)
    ImGui::SetWindowPos("Test window",ImVec2(50,50));

    // Rendering
    ImGui::Render();
    SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, 
                                 io.DisplayFramebufferScale.y);
    SDL_SetRenderDrawColor(renderer, 
        (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), 
        (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
    SDL_RenderPresent(renderer);
  }

  // Cleanup
  ImGui_ImplSDLRenderer_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
