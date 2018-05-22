cmake_minimum_required(VERSION 3.6)
option(CLANG_TIDY_ENABLE
  "If the command clang-tidy is avilable, tidy source files.\
Turn this off if the build time is too slow."
  ON)
find_program(CLANG_TIDY_EXE clang-tidy)

function(clang_tidy target)
  if(CLANG_TIDY_EXE)
    if(CLANG_TIDY_ENABLE)
      message(STATUS "Enable Clang-Tidy ${target}")
      set_target_properties(${target} PROPERTIES
        C_CLANG_TIDY "${CLANG_TIDY_EXE};-fix"
        CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-fix")
      endif()
  endif()
endfunction()
