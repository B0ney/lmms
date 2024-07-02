<<<<<<< HEAD
include(BuildPlugin)
build_plugin(granularpitchshifter
	GranularPitchShifterEffect.cpp
	GranularPitchShifterControls.cpp
	GranularPitchShifterControlDialog.cpp
	MOCFILES
	GranularPitchShifterControls.h
	GranularPitchShifterControlDialog.h
	EMBEDDED_RESOURCES *.png)
=======
INCLUDE(BuildPlugin)

BUILD_PLUGIN(granularpitchshifter GranularPitchShifter.cpp GranularPitchShifterControls.cpp GranularPitchShifterControlDialog.cpp MOCFILES GranularPitchShifterControls.h GranularPitchShifterControlDialog.h EMBEDDED_RESOURCES artwork.png logo.png)
>>>>>>> 13d697acf422b87e1a2d38aa59d78ca085bb1d5e
