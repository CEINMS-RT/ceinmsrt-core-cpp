# TCLAP Command line interface for C++ and CMake

This repository contains a mirror of the source of the TCLAP project from http://tclap.sourceforge.net/

As a bonus this version is wrapped by CMake.

## Install

Make a CMake install with:

```shell
mkdir build && cd build
cmake ..
cmake --install .
```

## Usage

After installing, you can use the library with:

```cmake
find_package(tclap)

target_link_libraries(myapplication PUBLIC tclap)
```

Or you can include the Git submodule and include the target directly:

```cmake
# `tclap` is the submodule, omit the project file and instead use the source lists directly
add_subdirectory(tclap/include)

target_link_libraries(myapplication PUBLIC tclap)
```
