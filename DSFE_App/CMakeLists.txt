# CMakeLists.txt for the DSFE_App
add_subdirectory(DSFE_Core)
option(DSFE_ENABLE_GUI "Enable GUI module" ON)
if(DSFE_ENABLE_GUI)
    add_subdirectory(DSFE_GUI)
endif()
add_subdirectory(DSFE_Engine)