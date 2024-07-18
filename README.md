CEINMS Real-Time
================

[![CodeFactor](https://www.codefactor.io/repository/bitbucket/ctw-bw/ceinms-rt/badge?s=05335d0cc499318ffdbe7be5e9b5637b56acb8bb)](https://www.codefactor.io/repository/bitbucket/ctw-bw/ceinms-rt)

GUI:
http://www.qcustomplot.com

## Compile

### Build

Build CEINMS (and each of its plugins) with:

```shell
mkdir build
cd build
cmake ..
cmake --build .
```

Instead of running the CMake command, you could use the CMake GUI. 
On Windows, when building with Visual Studio, you can open the generated VS solution instead 
running the build command.

In case installed requirements cannot be found by CMake, first try to add appending the paths
to the `CMAKE_PREFIX_PATH` variable like:

```shell
cmake .. -DCMAKE_PREFIX_PATH="/my/qt/install;/and/my/glew/install"
```

### Using Docker

A Dockerfile is provided that contains the dependencies to build CEINMS. Additionally, the image contains the tools needed to build and debug remotely.

 1. Build the docker image: `sudo docker build -t ceinms .`
    * `-t` provides the tag of the image
 2. Create a container from the image: `sudo docker run -it -d --name ceinms -v /full/path/to/ceinms-rt:/ceinms-rt -p127.0.0.1:2222:22 ceinms`
    * `-it` sets it in interactive mode
    * `-d` sets it in detach mode (keep running without terminal)
    * `--name` provides a name for the container
    * `-v` to mount a local directory
    * `-p` to forward a port
 3. Now start the container with `sudo docker start ceinms`

You will only need to build the image and container once (until it's deleted).

You should now be able to SSH into the container at port `2222` with `root:Docker` and `debugger:pwd`.  
You can set your IDE to use the container for building. In CLion for example you can create a toolchain
under settings for a Remote Host.

The Docker dependencies will now look like:

 * Local (unpublished) image called `ceinms`, in which you do your actual build.
   * [be1et/ceinms-platform](https://hub.docker.com/r/be1et/ceinms-platform), a custom image with additional build dependencies
     * [stanfordnmbl/opensim-cpp](https://hub.docker.com/r/stanfordnmbl/opensim-cpp), the official OpenSim image for C++
<<<<<<< HEAD
       * [stanfordnmbl/opensim-deps](https://hub.docker.com/r/stanfordnmbl/opensim-deps), the official OpenSim dependencies image
=======
       * [stanfordnmbl/opensim-deps](https://hub.docker.com/r/stanfordnmbl/opensim-deps), the official OpenSim dependencies image

## Packaging

Building CEINMS is tricky because of the large number of dependencies. Instead, you can share the compiled application 
using CPack.

After building the application, run cpack from inside the build directory:

```shell
cpack -G ZIP
```

Use `ZIP` for create an archive of the binaries. Use `NSIS64` or `WIX` to create a Windows installer.

The packager will try to find the runtime dependencies, but it searches in different places and will not always succeed.
If you see a warning about a missing runtime library, try adding the directory of that library to the variable 
`PACKAGE_DEPENDENCIES_DIRECTORIES` and run the `cmake` configuration again, before trying `cpack` once more.

Afterwards you can find the packaged result in the build directory.
