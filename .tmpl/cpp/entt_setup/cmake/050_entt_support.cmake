# Get EnTT, 
include(FetchContent)

SET(ENTT_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/entt)
SET(ENTT_INC_DIR ${ENTT_SRC_DIR}/src)
FetchContent_Declare(
    entt
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG v3.13.2
    GIT_PROGRESS TRUE
    SOURCE_DIR ${ENTT_SRC_DIR}
)
FetchContent_MakeAvailable(entt)

SET(INCLUDE_DIR ${INCLUDE_DIR} ${ENTT_INC_DIR} )
