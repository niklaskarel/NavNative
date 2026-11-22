cmake_minimum_required(VERSION 3.21)
project(NavNativeLab VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(NAV_BUILD_GUI "Build Qt route visualizer" ON)
set(NAVCore_sources
  src/RouteGeometry.cpp
  src/RouteTracker.cpp
)
set(NAVCore_headers
  include/nav/RouteTracker.h
  include/nav/RouteGeometry.h
)
add_library(NAVCore ${NAVCore_sources} ${NAVCore_headers})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${NAVCore_sources})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "Header Files" FILES ${NAVCore_headers})
target_include_directories(NAVCore PUBLIC include)
if(MSVC)
  target_compile_options(NAVCore PRIVATE /W4)
else()
  target_compile_options(NAVCore PRIVATE -Wall -Wextra -Wpedantic)
endif()
option(BUILD_TESTING "Build the navigation geometry tests" ON)
if(BUILD_TESTING)
  enable_testing()
  add_executable(NAVTests tests/TrackerTest.cpp)
  target_link_libraries(NAVTests PRIVATE NAVCore)
  find_package(GTest CONFIG QUIET)
  if(TARGET GTest::gtest_main)
    target_link_libraries(NAVTests PRIVATE GTest::gtest_main)
  else()
    include(FetchContent)
    if(POLICY CMP0135)
      cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(googletest
      URL https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.16.0
      URL_HASH SHA256=78c676fc63881529bf97bf9d45948d905a66833fbfa5318ea2cd7478cb98f399
      SOURCE_SUBDIR googletest/src
    )
    FetchContent_MakeAvailable(googletest)
    # Compile GoogleTest into NAVTests so the solution has one test project.
    set(gtest_dir "${googletest_SOURCE_DIR}/googletest")
    set(gtest_sources "${gtest_dir}/src/gtest-all.cc" "${gtest_dir}/src/gtest_main.cc")
    target_sources(NAVTests PRIVATE ${gtest_sources})
    source_group("GoogleTest" FILES ${gtest_sources})
    target_include_directories(NAVTests SYSTEM PRIVATE "${gtest_dir}" "${gtest_dir}/include")
    find_package(Threads REQUIRED)
    target_link_libraries(NAVTests PRIVATE Threads::Threads)
  endif()
  add_test(NAME navigation_geometry COMMAND NAVTests)
endif()
set(viewer_sources
  app/main.cpp
  app/ViewerWindow.cpp
  app/MapCanvas.cpp
  app/CityRenderer.cpp
  app/DriveSimulation.cpp
  app/CsvRecorder.cpp
)
set(viewer_headers
  app/ViewerWindow.h
  app/MapCanvas.h
  app/CityMap.h
  app/CityRenderer.h
  app/DriveSimulation.h
  app/CsvRecorder.h
)
if(NAV_BUILD_GUI)
  find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets)
  qt_add_executable(NAVViewer ${viewer_sources} ${viewer_headers})
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/app" PREFIX "Source Files" FILES ${viewer_sources})
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/app" PREFIX "Header Files" FILES ${viewer_headers})
  qt_add_resources(NAVViewer map_assets
    PREFIX "/map"
    FILES
      app/assets/BuildingBlue.png
      app/assets/BuildingGreen.png
      app/assets/BuildingRed.png
  )
  target_link_libraries(NAVViewer PRIVATE NAVCore Qt6::Widgets)
  if(WIN32)
    add_custom_command(TARGET NAVViewer POST_BUILD
      COMMAND Qt6::windeployqt --no-translations "$<TARGET_FILE:NAVViewer>")
  endif()
endif()

