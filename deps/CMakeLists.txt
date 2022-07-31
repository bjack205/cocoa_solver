
macro(AddFmtLib)
  list(APPEND CMAKE_MESSAGE_CONTEXT fmt)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG d141cdbeb0fb422a3fb7173b285fd38e0d1772dc # version 8.0.1
  )
  FetchContent_MakeAvailable(fmt)
  list(POP_BACK CMAKE_MESSAGE_CONTEXT fmt)
endmacro()

macro(AddGoogleTest)
  list(APPEND CMAKE_MESSAGE_CONTEXT gtest)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  include(GoogleTest)
  include(CTest)
  list(POP_BACK CMAKE_MESSAGE_CONTEXT gtest)
endmacro()

macro(AddQdldl)
  list(APPEND CMAKE_MESSAGE_CONTEXT qdldl)
  FetchContent_Declare(
    qdldl
    GIT_REPOSITORY https://github.com/osqp/qdldl
    GIT_TAG 29d140419a3bec20d860052d73ba2be927faf5a1 # version 0.1.6
  )
  FetchContent_MakeAvailable(qdldl)
  list(POP_BACK CMAKE_MESSAGE_CONTEXT qdldl)
endmacro()