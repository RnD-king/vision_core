#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "shared_vision_core::shared_vision_core" for configuration "Release"
set_property(TARGET shared_vision_core::shared_vision_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(shared_vision_core::shared_vision_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libshared_vision_core.so"
  IMPORTED_SONAME_RELEASE "libshared_vision_core.so"
  )

list(APPEND _cmake_import_check_targets shared_vision_core::shared_vision_core )
list(APPEND _cmake_import_check_files_for_shared_vision_core::shared_vision_core "${_IMPORT_PREFIX}/lib/libshared_vision_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
