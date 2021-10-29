# Robotino2 Wrapper

There is Festo [OpenRobotiAPI](https://wiki.openrobotino.org/index.php?title=OpenRobotinoAPI) wrapper.

## Requirements

- CMake v3.0 or later
- OpenRobotinoAPI v1.8.31 or later
- MSVC v19 or later

## Configure, build and install library

### Windows 10 x86

```
# Create build directory
mkdir build

# Configure as static library
cmake -S robotino2-wrapper/ -B build/ -G "Visual Studio 15"

# Build
cmake --build build/

# Install
sudo cmake --install build/
```

You can also build and install shared library:
```
# Configure as shared library
cmake -S robotino2-wrapper/ -B build/ -DBUILD_SHARED_LIBS=ON
```

You can build and install tests:
```
# Configure as static library
cmake -S robotino2-wrapper/ -B build/ -DBUILD_TESTS=ON -DINSTALL_TESTS=ON
```

## Using Robotino2 Wrapper with CMake

Add this strings in your CMakeLists.txt file:
```
find_package(Robotino2 1.0 REQUIRED)
target_link_libraries(<ProjectName> Robotino2Lib)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${Robotino2_INCLUDE_DIRS})
```