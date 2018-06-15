cmake_minimum_required(VERSION 3.7)

# Include misc modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompileOptions)
include(EnableMeasureTime)
include(EnableCcache)
include(DoClangTidy)
include(FormatFilesWithClangFormat)

# Set project info
set(MY_PROJECT_NAME   "optcarrot_cpp")
set(MY_VERSION_MAJOR  "0")
set(MY_VERSION_MINOR  "1")
set(MY_VERSION_PATCH  "0")
set(MY_VERSION        "${MY_VERSION_MAJOR}.${MY_VERSION_MINOR}.${MY_VERSION_PATCH}")
project(${MY_PROJECT_NAME}
  VERSION ${MY_VERSION}
  LANGUAGES C CXX
)

# Set output path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Find SDL
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2 sdl>=2.0.0)
pkg_search_module(SDL2IMAGE REQUIRED SDL2_image SDL_image>=2.0.0)

# Build library
file(GLOB_RECURSE SRCS "source/*.cpp" "source/*.h" "include/*.h")
add_library(${PROJECT_NAME} SHARED ${SRCS})
target_compile_options(${PROJECT_NAME}
  PRIVATE
  ${DEFAULT_COMPILE_OPTIONS}
)
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${SDL2_INCLUDE_DIRS}" "${SDL2IMAGE_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} "${SDL2_LIBRARIES}" "${SDL2IMAGE_LIBRARIES}")
clang_format(${PROJECT_NAME} ${SRCS})
clang_tidy(${PROJECT_NAME})

# Build executable
add_executable(${PROJECT_NAME}-exe main.cpp)
set_target_properties(${PROJECT_NAME}-exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}-exe PRIVATE ${DEFAULT_COMPILE_OPTIONS})
target_link_libraries(${PROJECT_NAME}-exe ${PROJECT_NAME} stdc++fs)
clang_format(${PROJECT_NAME}-exe main.cpp)
#clang_tidy(${PROJECT_NAME}-exe)


# Build tests if option enabled
#include(CTest)
#if(BUILD_TESTING)
#  message(STATUS "Build testing")
#  enable_testing()
#  add_subdirectory(test)
#endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

