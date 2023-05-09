find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQ libmosquitto REQUIRED)

SET ( INCLUDE_DIR
        ${INCLUDE_DIR}
        ${MOSQ_INCLUDEDIR}
        )
SET ( LIBRARY_LISTS ${LIBRARY_LISTS}
  ${MOSQ_LIBRARIES}
  )
