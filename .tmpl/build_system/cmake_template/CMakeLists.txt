CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
PROJECT ( WW_ProjectName_WW )

# Instruct CMake to run moc automatically when needed

SET ( PROJECT_VERSION_MAJOR WW_MajorVer_WW)
SET ( PROJECT_VERSION_MINOR WW_MinorVer_WW)
 
SET ( CMAKE_BUILD_TYPE       Debug )
SET ( CMAKE_VERBOSE_MAKEFILE true )

# :x: For extention of cmake file 
# :x: you can add *.cmake file, under ./cmake/ directory
FILE( GLOB INCLUDE_CMAKE_FILES cmake/*.cmake)
FOREACH( FILE ${INCLUDE_CMAKE_FILES})
  include( ${FILE})
ENDFOREACH()

SET ( OUTPUT_ELF
  "${CMAKE_PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.out"
  )

SET ( INCLUDE_DIR
        include
        )
# Sources for library build
FILE( GLOB LIBSRC_FILES libsrc/*.cpp)

# sources for exec
FILE( GLOB SRC_FILES src/*.cpp)

# sources for testing
FILE( GLOB TEST_FILES test/*.cpp)

 
SET ( CMAKE_C_COMPILER "gcc" )
SET ( CMAKE_CXX_COMPILER "g++" )
 
#INCLUDE_DIRECTORIES ( include )
 
# common compile options
ADD_COMPILE_OPTIONS ( -Wall 
                    # -Werror
                       )
# Link library lists (-l)
SET ( LIBRARY_LISTS ${LIBRARY_LISTS} 
# add the libraries what you want
# ex) SDL2,  Qt5::Widgets ...
    )
 
# commone link library directories (-L)
#LINK_DIRECTORIES ( 
#  ${CMAKE_BUILD_TYPE}/lib )
 
# "Debug" 형상 한정 컴파일 옵션, 링크 옵션
SET ( CMAKE_C_FLAGS_DEBUG   "-O0 -g3" )
SET ( CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -pthread" )
SET ( CMAKE_EXE_LINKER_FLAGS_DEBUG "" )
 
# "Release" 형상 한정 컴파일 옵션, 링크 옵션
SET ( CMAKE_C_FLAGS_RELEASE "-O2" )
SET ( CMAKE_CXX_FLAGS_RELEASE "-O2 -pthread" )
SET ( CMAKE_EXE_LINKER_FLAGS_RELEASE "" )
 
SET ( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BUILD_TYPE} )
SET ( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BUILD_TYPE}/lib )
SET ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BUILD_TYPE}/lib )
 
# build Library
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
ADD_LIBRARY( ${CMAKE_PROJECT_NAME} SHARED ${LIBSRC_FILES} )

# build exec
ADD_EXECUTABLE( ${OUTPUT_ELF} ${SRC_FILES} )
TARGET_INCLUDE_DIRECTORIES (${OUTPUT_ELF} PUBLIC ${INCLUDE_DIR} )
TARGET_LINK_LIBRARIES(${OUTPUT_ELF} ${CMAKE_PROJECT_NAME} )
TARGET_LINK_LIBRARIES(${OUTPUT_ELF} ${LIBRARY_LISTS} )

# build Test
SET (TEST_RUNNER test_${OUTPUT_ELF})
ADD_EXECUTABLE( ${TEST_RUNNER} ${TEST_FILES} )
TARGET_INCLUDE_DIRECTORIES (${TEST_RUNNER} PUBLIC ${INCLUDE_DIR} )
TARGET_LINK_LIBRARIES(${TEST_RUNNER} 
                       -pthread
                       gtest 
                       ${CMAKE_PROJECT_NAME})


