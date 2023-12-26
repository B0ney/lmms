INCLUDE(BuildPlugin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
BUILD_PLUGIN(limiter Limiter.cpp LimiterControls.cpp LimiterControlDialog.cpp MOCFILES Limiter.h LimiterControls.h LimiterControlDialog.h OversamplingHelpers.h EMBEDDED_RESOURCES artwork.png logo.png)
TARGET_LINK_LIBRARIES(limiter hiir)