SET(SCENES_DIR libsrc/scenes)                                   
                                                                  
FILE( GLOB_RECURSE SCENE_FILES  libsrc/scenes/*.cpp)

SET(INCLUDE_DIR ${INCLUDE_DIR} ${SCENE_DIR} )
SET(LIBSRC_FILES ${LIBSRC_FILES} ${SCENE_FILES}) 

