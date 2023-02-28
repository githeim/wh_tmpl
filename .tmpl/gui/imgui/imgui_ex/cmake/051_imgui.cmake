# Get imgui v1.89.3 
include(FetchContent)
 FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.89.3.zip 
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui
 )
FetchContent_MakeAvailable(imgui)

SET(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)

SET(IMGUI_SRC 
  ${IMGUI_DIR}/imgui.cpp 
  ${IMGUI_DIR}/imgui_demo.cpp 
  ${IMGUI_DIR}/imgui_draw.cpp 
  ${IMGUI_DIR}/imgui_tables.cpp 
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp 
  ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer.cpp
  )
SET(INCLUDE_DIR ${INCLUDE_DIR} ${IMGUI_DIR} ${IMGUI_DIR}/backends)
SET(SRC_FILES ${SRC_FILES} ${IMGUI_SRC})
