INCLUDE(BuildPlugin)

BUILD_PLUGIN(architect2 Architect2.cpp Architect2.h MOCFILES Architect2.h EMBEDDED_RESOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.png")
