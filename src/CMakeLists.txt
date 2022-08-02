set(CLANG_TIDY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-tidy)
if (COCOA_RUN_CLANG_TIDY AND (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  message(STATUS "Running clang-tidy on source directory.")
  set(CMAKE_C_CLANG_TIDY
    clang-tidy;
    -export-fixes=tidy-fixes.yaml;
    -config-file=${CLANG_TIDY_FILE}
    )
endif()
cmake_print_variables(CLANG_TIDY_FILE)

add_executable(
  main
  main.c
)
target_link_libraries(
  main
  PRIVATE
  qdldlstatic
  slap::slap
)

add_library(
  cocoa
  cocoa.c
  cocoa.h
  cocoa_types.h
  cocoa_mpc.c cocoa_mpc.h cocoa_advanced.c cocoa_advanced.h cocoa_qp.c cocoa_qp.h)
target_link_libraries(
  cocoa
  PRIVATE
  qdldlstatic
  slap::slap
)
