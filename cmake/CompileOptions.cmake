cmake_minimum_required(VERSION 3.8)
enable_language(CXX)
enable_language(C)
set(CMAKE_CXX_STANDARD 17) # need cmake >3.8
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(DEFAULT_COMPILE_OPTIONS)
# enable warnings
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
  set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
    -Wall
    -Weverything
    -Wno-c++98-compat
    -Wno-c++98-compat-pedantic
    )
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU")
  set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
    -pedantic
    -Wall
    -Wextra
    -Wcast-align
    -Wcast-qual
    -Wdisabled-optimization
    -Wformat=2
    -Winit-self
    -Wlogical-op
    -Wmissing-declarations
    -Wmissing-include-dirs
    -Wredundant-decls
    -Wshadow
    -Wsign-conversion
    -Wstrict-overflow=5
    -Wswitch-default
    -Wundef
    )
elseif(CMAKE_C_COMPILER_ID MATCHES "MSVC")
  set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
    /W4
    )
endif()

# set message to color
if(CMAKE_GENERATOR MATCHES "Ninja")
  set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
    "$<$<C_COMPILER_ID:Clang>:-fcolor-diagnostics>"
    "$<$<C_COMPILER_ID:GNU>:-fdiagnostics-color=always>"
    )
endif()

