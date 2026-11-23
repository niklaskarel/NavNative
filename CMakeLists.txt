cmake_minimum_required(VERSION 3.21)
project(NavNativeLab VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(NAV_BUILD_GUI "Build Qt route visualizer" ON)
find_package(Threads REQUIRED)

set(core_sources
  src/Geometry/RouteGeometry.cpp
  src/Routing/LocalProjection.cpp
  src/Routing/OpenStreetMap.cpp
  src/Routing/RoadGraph.cpp
  src/Routing/RoutePlanner.cpp
  src/Gnss/NmeaRmc.cpp
  src/Tracking/RouteTracker.cpp
  src/Simulation/DriveSimulation.cpp
  src/Simulation/SimulatedGnssSource.cpp
  src/Session/RouteSession.cpp
  src/Session/TrackingWorker.cpp
  src/Logger/CsvRecorder.cpp
  src/Logger/Log.cpp
  src/Logger/PositionLogger.cpp
)
set(core_headers
  include/nav/Geometry/RouteGeometry.h
  include/nav/Geometry/NavigationTypes.h
  include/nav/Routing/LocalProjection.h
  include/nav/Routing/OpenStreetMap.h
  include/nav/Routing/RoadGraph.h
  include/nav/Routing/RoutePlanner.h
  include/nav/Gnss/NmeaRmc.h
  include/nav/Tracking/RouteTracker.h
  include/nav/Simulation/DriveSimulation.h
  include/nav/Simulation/SimulatedGnssSource.h
  include/nav/Session/RouteSession.h
  include/nav/Session/TrackingWorker.h
  include/nav/Session/ProtectedQueue.h
  include/nav/Logger/CsvRecorder.h
  include/nav/Logger/Log.h
  include/nav/Logger/PositionLogger.h
)
add_library(NAVCore STATIC ${core_sources} ${core_headers})
target_include_directories(NAVCore PUBLIC include)
target_link_libraries(NAVCore PRIVATE Threads::Threads)
if(MSVC)
  target_compile_options(NAVCore PRIVATE /W4)
else()
  target_compile_options(NAVCore PRIVATE -Wall -Wextra -Wpedantic)
endif()
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${core_sources})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include/nav" PREFIX "Header Files" FILES ${core_headers})

find_package(pugixml 1.16 CONFIG QUIET)
if(TARGET pugixml::pugixml)
  target_link_libraries(NAVCore PRIVATE pugixml::pugixml)
else()
  include(FetchContent)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  FetchContent_Declare(pugixml
    URL https://codeload.github.com/zeux/pugixml/tar.gz/refs/tags/v1.16
    URL_HASH SHA256=357bcab8877dc9943f355d3a72daba1b053238ba955f50fa81586afb65090219
    SOURCE_SUBDIR src
  )
  FetchContent_MakeAvailable(pugixml)
  target_sources(NAVCore PRIVATE "${pugixml_SOURCE_DIR}/src/pugixml.cpp")
  target_include_directories(NAVCore SYSTEM PRIVATE "${pugixml_SOURCE_DIR}/src")
  source_group("pugixml" FILES "${pugixml_SOURCE_DIR}/src/pugixml.cpp")
endif()

option(BUILD_TESTING "Build the navigation geometry tests" ON)
if(BUILD_TESTING)
  enable_testing()
  add_executable(NAVTests tests/TrackerTest.cpp tests/NavigationInputTest.cpp tests/RouteSessionTest.cpp tests/PositionLoggerTest.cpp)
  target_link_libraries(NAVTests PRIVATE NAVCore Threads::Threads)
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
  endif()
  add_test(NAME navigation_geometry COMMAND NAVTests)
endif()
set(viewer_sources
  app/main.cpp
  app/ViewerWindow.cpp
  app/MapCanvas.cpp
)
set(viewer_headers
  app/ViewerWindow.h
  app/MapCanvas.h
)
if(NAV_BUILD_GUI)
  find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets)
  set(viewer_form app/ViewerWindow.ui)
  add_executable(NAVViewer ${viewer_sources} ${viewer_headers} ${viewer_form})
  set_target_properties(NAVViewer PROPERTIES AUTOMOC ON AUTOUIC ON)
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/app" PREFIX "Source Files" FILES ${viewer_sources})
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/app" PREFIX "Header Files" FILES ${viewer_headers})
  source_group("Forms" FILES ${viewer_form})
  file(GENERATE
    OUTPUT "$<TARGET_FILE_DIR:NAVViewer>/Eixample.osm"
    INPUT "${CMAKE_CURRENT_SOURCE_DIR}/app/assets/Eixample.osm"
  )
  target_link_libraries(NAVViewer PRIVATE NAVCore Qt6::Widgets)
  if(WIN32)
    add_custom_command(TARGET NAVViewer POST_BUILD
      COMMAND Qt6::windeployqt --no-translations "$<TARGET_FILE:NAVViewer>")
  endif()
endif()

