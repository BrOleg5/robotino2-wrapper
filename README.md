# Robotino2 Wrapper

There is Festo [OpenRobotinoAPI](https://wiki.openrobotino.org/index.php?title=OpenRobotinoAPI) wrapper.

## Requirements

- CMake v3.0 or later
- OpenRobotinoAPI v1.8.31 or later
- compiler: MSVC v19 (Visual Studio 2017) or later

Optional:

- Doxygen v1.8.0 or later

## Configure, build and install library with CMake

### Windows 10 x86

```bash
# Create build directory
mkdir build

cmake -S robotino2-wrapper -B build

# Build certain configuration <cfg>: Debug (default), Release
cmake --build build --config <cfg>

# Install certain configuration <cfg>: Debug (default), Release
# <prefix> is installation path (default SystemPartition:\Program Files (x86)\<project name>)
cmake --install build --config <cfg> --prefix <prefix>
```

You can build and install test programs. For this add option `-D BUILD_TESTS=ON`

```bash
# Configure
cmake -S robotino2-wrapper -B build -D BUILD_TESTS=ON
```

To build Doxygen documentation, use target `doxygen` and configuration Release

```bash
cmake --build build --config Release --target doxygen --prefix <prefix>
cmake --install build --config Release --target doxygen --prefix <prefix>
```

## Using Robotino2 Wrapper with CMake

Add this strings in your CMakeLists.txt file:

```bash
find_package(Robotino2Wrapper 1.0 REQUIRED)
target_link_libraries(<ProjectName> robotino2wrapper)
# if necessary, add include directories to target
# target_include_directories(<ProjectName> ${Robotino2Wrapper_INCLUDE_DIRS})
```
