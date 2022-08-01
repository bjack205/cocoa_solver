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
