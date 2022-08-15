function(add_cocoa_test testname)
  add_executable(${testname}_test
    ${testname}_test.cpp
    )
  target_link_libraries(${testname}_test
    PRIVATE
    gtest_main
    fmt::fmt
    cocoa
    )
  gtest_discover_tests(${testname}_test)
endfunction()

add_cocoa_test(basic)