# set the SDL2 library                 
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

SET ( LIBRARY_LISTS ${LIBRARY_LISTS}   
  ${SDL2_LIBRARIES}
  SDL2_ttf
  SDL2_image
  )                                  
