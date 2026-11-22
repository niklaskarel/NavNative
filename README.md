# NavNative Lab

NavNative Lab is a C++20 route-tracking project with a Qt 6 desktop visualizer and CSV recording. The tracking library projects GPS observations onto the nearest segment of a route, estimates distance traveled along the route, and reports whether the observation is off route. The demo uses synthetic GPS observations and local Cartesian coordinates in meters.

## Prerequisites

- CMake 3.21 or later.
- Visual Studio 2022 with the **Desktop development with C++** workload for the Visual Studio build.
- Qt 6.5 or later with Qt Widgets for the viewer. Use an x64 MSVC Qt kit with the Visual Studio preset.
- Ninja and a C++20 compiler if you use the Ninja preset. Choose a Qt kit that matches your compiler and architecture.
- The test target uses GoogleTest. CMake downloads the pinned version on first configure if GoogleTest is unavailable locally. Set `-DBUILD_TESTING=OFF` to build just the viewer and library.

## Build and run on Windows

Run these commands in PowerShell from the project root. Replace the example path with the directory of your installed Qt kit.

```powershell
cmake --preset vs-qt -DCMAKE_PREFIX_PATH="C:/Qt/6.8.3/msvc2022_64"
cmake --build --preset vs-qt
ctest --preset vs-qt
.\build\vs-qt\Debug\NAVViewer.exe
```

CMake embeds the building sprites and deploys the viewer's Qt runtime files when it builds the Windows executable. The viewer shows a simulated route through an Eixample-inspired street grid. Controls adjust vehicle speed and GPS noise, inject a GPS offset, and pause or reset the simulation.

Click **Record CSV** to choose an output file, then **Stop Recording** to close it. The CSV contains elapsed recording time, true and observed positions, the selected match, true and estimated progress, distance from the route, and the off-route flag.

To work in Visual Studio, open the project root with **File > Open > Folder**, select the `vs-qt` configure preset, and build. Set `NAVViewer.exe` as the startup target and press **F5** to debug. You can store your Qt kit path in `CMakeUserPresets.json` using a local preset that inherits from `vs-qt`.

## Build with Ninja

From a shell with your compiler and Ninja on the path, pass the path to your matching Qt kit.

```powershell
cmake --preset ninja-qt -DCMAKE_PREFIX_PATH="<path-to-qt-kit>"
cmake --build --preset ninja-qt
ctest --preset ninja-qt
.\build\ninja-qt\NAVViewer.exe
```

## Build the library and tests

Set `NAV_BUILD_GUI=OFF` to build the C++ library and tests with CMake and MSVC.

```powershell
cmake -S . -B build/core -G "Visual Studio 17 2022" -A x64 -DNAV_BUILD_GUI=OFF
cmake --build build/core --config Debug
ctest --test-dir build/core -C Debug --output-on-failure
```

## Project layout

```text
app/
  main.cpp           Qt application entry point
  ViewerWindow.*    Controls, simulation timer, and GPS readings
  DriveSimulation.* Vehicle motion and simulated GPS fixes
  CsvRecorder.*     Writes viewer frames to CSV
  MapCanvas.*       Route, GPS history, and position markers
  CityMap.h         Grid dimensions and viewer route coordinates
  CityRenderer.*    Streets, crosswalks, and building sprites
  assets/           Building PNGs embedded by CMake
include/nav/
  RouteTracker.h         Route tracker API and geometry types
  RouteGeometry.h  Geometry helper declarations
src/
  RouteTracker.cpp       Nearest-segment matching and route progress
  RouteGeometry.cpp Segment projection and route interpolation
tests/
  TrackerTest.cpp Geometry tests
CMakeLists.txt      Build targets and Qt resources
CMakePresets.json  Shared build presets
.clang-format      Shared C++ formatting rules
```

| Target | Source | Purpose |
| --- | --- | --- |
| `NAVCore` | `include/nav/`, `src/` | Route geometry and nearest-segment tracking |
| `NAVViewer` | `app/main.cpp` | Interactive Qt route simulation |
| `NAVTests` | `tests/TrackerTest.cpp` | CTest coverage for projection, route progress, endpoints, duplicate vertices, and invalid routes |


## TODO

- [ ] Add a test that captures the progress jump on nearby parallel segments.
- [ ] Name the 25 m off-route threshold and test its boundary behavior.
- [ ] Compare the nearest-segment baseline with a candidate rule that uses the previous fix and elapsed time. Check how it behaves when the car reverses or location fixes resume after a gap.
