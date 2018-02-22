# bump version here
set(vtsoffscreen_VERSION 1.1)

define_module(LIBRARY vtsoffscreen=${vtsoffscreen_VERSION}
  DEPENDS geo>=1.28 optics>=1.1 geometry>=1.10 imgproc>=1.23
  glsupport>=1.0 dbglog>=1.4
  VtsBrowser VtsRenderer OpenCV
  )

set(vtsoffscreen_SOURCES
  snapper.hpp snapper.cpp
  )

add_library(vtsoffscreen ${vtsoffscreen_SOURCES})
target_link_libraries(vtsoffscreen
  ${MODULE_LIBRARIES} ${CMAKE_DL_LIBS})
buildsys_target_compile_definitions(vtsoffscreen ${MODULE_DEFINITIONS})
buildsys_library(vtsoffscreen)
