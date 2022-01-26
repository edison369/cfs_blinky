project(CFE_BLINKY C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${blinky_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(blinky fsw/src/blinky.c)
