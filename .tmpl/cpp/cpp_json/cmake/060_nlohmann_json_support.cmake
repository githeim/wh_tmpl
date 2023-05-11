find_package(nlohmann_json  REQUIRED)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(MOSQ libmosquitto REQUIRED)
# 
# SET ( INCLUDE_DIR
#         ${INCLUDE_DIR}
#         ${MOSQ_INCLUDEDIR}
#         )
SET ( LIBRARY_LISTS ${LIBRARY_LISTS}
  nlohmann_json::nlohmann_json
   )
