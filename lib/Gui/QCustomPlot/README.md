# QCustomPlot

This is a repository of Emanuel Eichhammer's library, see https://www.qcustomplot.com.  
We are not the owners or authors of this library! We are simply hosting the code here
keep our own applications more clean.

## Build

To build and install the library, simply run:

```shell
mkdir build && cd build
cmake ..
cmake --build .
cmake --install .
```

Now in your project you can include it with:

```cmake
find_package(qcustomplot)

target_link_libraries(myapplication qcustomplot)
```

## Include

You can also literally include the library in your Project as a Git submodule:

```shell
git submodule add https://github.com/ET-BE/QCustomPlot.git 
```

Then in your CMakeLists, add something like:

```cmake
add_subdirectory("<path/to/QCustomPlot/qcustomplot")

target_link_libraries(myapplication QCustomPlot)
```

Make sure to skip the project `CMakeLists.txt` of this library and only include the
source one.
