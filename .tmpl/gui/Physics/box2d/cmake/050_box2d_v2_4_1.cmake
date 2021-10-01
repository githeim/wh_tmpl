include(ExternalProject)
# Get box2d 2.4.1 
ExternalProject_Add(box2d_v2_4_1
  URL https://github.com/erincatto/box2d/archive/refs/tags/v2.4.1.zip
  URL_MD5 02f330ca8585dbe621966a58f77c2b67
  TIMEOUT 600
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/box2d
    -DCMAKE_BUILD_TYPE=Release
)

SET ( LIBRARY_LISTS ${LIBRARY_LISTS} 
  box2d
)
SET ( DEPENDENCY_LIST ${DEPENDENCY_LIST} 
  box2d_v2_4_1
)

SET ( INCLUDE_DIR
        ${INCLUDE_DIR} 
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/box2d/include
  )
LINK_DIRECTORIES ( 
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/box2d/lib )

