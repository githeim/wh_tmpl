# Get imgui v1.89.3 
include(FetchContent)
SET(IMGUI_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui)
FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.89.3.zip 
    SOURCE_DIR ${IMGUI_SRC_DIR}
 )
FetchContent_MakeAvailable(imgui)

SET(IMGUI_DIR ${IMGUI_SRC_DIR})

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
SET(LIBSRC_FILES ${LIBSRC_FILES} ${IMGUI_SRC})
