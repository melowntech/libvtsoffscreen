# bump version here
set(vtsoffscreen_VERSION 1.1)

# clear variables
set(EXTRA_DEPENDS)
set(EXTRA_DEFINITIONS)

if(MODULE_optics_FOUND)
  message(STATUS "vtsoffscreen: compiling in optics support")
  list(APPEND EXTRA_DEPENDS optics>=1.1)
  list(APPEND EXTRA_DEFINITIONS VTSOFFSCREEN_HAS_OPTICS=1)
else()
    message(STATUS "vtsoffscreen: compiling without optics support")
endif()

define_module(LIBRARY vtsoffscreen=${vtsoffscreen_VERSION}
  DEPENDS geo>=1.28 geometry>=1.10 imgproc>=1.23
  glsupport>=1.0 dbglog>=1.4
  VtsBrowser VtsRenderer OpenCV
  ${EXTRA_DEPENDS}
  DEFINITIONS ${EXTRA_DEFINITIONS}
  )

set(vtsoffscreen_SOURCES
  snapper.hpp snapper.cpp
  position.hpp
  )

add_library(vtsoffscreen ${vtsoffscreen_SOURCES})
target_link_libraries(vtsoffscreen
  ${MODULE_LIBRARIES} ${CMAKE_DL_LIBS})
target_compile_definitions(vtsoffscreen PRIVATE ${MODULE_DEFINITIONS})
buildsys_library(vtsoffscreen)
