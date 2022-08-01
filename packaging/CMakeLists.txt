include(CMakePackageConfigHelpers)

# Add dependencies to export set
install(
  TARGETS
  slap
  qdldlstatic
  EXPORT cocoa_export
)

# Install targets
install(EXPORT cocoa_export
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
  NAMESPACE cocoa::
  FILE cocoa.cmake
  COMPONENT cocoa
  )

# Install header files
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/src/   # final backslash copies contents of src/, not src/ itself
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cocoa
  FILES_MATCHING
  PATTERN "*.h"
)

# Write config and version file
install(FILES cocoaConfig.cmake
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  )
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/cocoaConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
  )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cocoaConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cocoaConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
  )