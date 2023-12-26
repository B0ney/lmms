INCLUDE(BuildPlugin)

BUILD_PLUGIN(mercury Mercury.cpp Mercury.h MOCFILES Mercury.h EMBEDDED_RESOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.png")
