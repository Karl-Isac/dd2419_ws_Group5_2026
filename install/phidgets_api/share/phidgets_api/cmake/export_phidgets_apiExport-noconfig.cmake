#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "phidgets_api::phidgets_api" for configuration ""
set_property(TARGET phidgets_api::phidgets_api APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(phidgets_api::phidgets_api PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libphidgets_api.so"
  IMPORTED_SONAME_NOCONFIG "libphidgets_api.so"
  )

list(APPEND _cmake_import_check_targets phidgets_api::phidgets_api )
list(APPEND _cmake_import_check_files_for_phidgets_api::phidgets_api "${_IMPORT_PREFIX}/lib/libphidgets_api.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
