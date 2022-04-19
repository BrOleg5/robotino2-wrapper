# Robotino2 Wrapper

There is Festo [OpenRobotinoAPI](https://wiki.openrobotino.org/index.php?title=OpenRobotinoAPI) wrapper.

## Requirements

- CMake v3.0 or later
- OpenRobotinoAPI v1.8.31 or later
- compiler: MSVC v19 or later (Visual Studio 2017)

## Configure, build and install library with CMake

### Windows 10 x86

```
# Create build directory
mkdir .\build\

# Configure. Build system generator <generator-name>: for MSVC 19 (Visual Studio 2017) is "Visual Studio 15".
# Command "cmake --help" print full lust of generators that are available on your platform
cmake -S .\robotino2-wrapper\ -B .\build\ -G <generator-name>

# Build certain configuration <cfg>: Debug (default), Release
cmake --build .\build\ --config <cfg>

# Install certain configuration <cfg>: Debug (default), Release
# <prefix> is installation path (default SystemPartition:\Program Files (x86)\<project name>)
cmake --install .\build\ --config <cfg> --prefix <prefix>
```

You can build and install test programs. For this add option `-D BUILD_TESTS=ON`.
```
# Configure
cmake -S .\robotino2-wrapper\ -B .\build\ -D BUILD_TESTS=ON
```

## Using Robotino2 Wrapper with CMake

Add this strings in your CMakeLists.txt file:
```
find_package(Robotino2 1.0 REQUIRED)
target_link_libraries(<ProjectName> Robotino2Lib)
# if necessary, add include directories to target
target_include_directories(<ProjectName> ${Robotino2_INCLUDE_DIRS})
```