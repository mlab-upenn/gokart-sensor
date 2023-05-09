#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "septentrio_gnss_driver::septentrio_gnss_driver_core" for configuration "Release"
set_property(TARGET septentrio_gnss_driver::septentrio_gnss_driver_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(septentrio_gnss_driver::septentrio_gnss_driver_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libseptentrio_gnss_driver_core.so"
  IMPORTED_SONAME_RELEASE "libseptentrio_gnss_driver_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS septentrio_gnss_driver::septentrio_gnss_driver_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_septentrio_gnss_driver::septentrio_gnss_driver_core "${_IMPORT_PREFIX}/lib/libseptentrio_gnss_driver_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
