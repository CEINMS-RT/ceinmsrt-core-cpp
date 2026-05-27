#!/bin/bash

# Function definition to install Boost
install_boost () {
    echo "Installing Boost..."
    cd
    sudo apt-get install -y build-essential python3 libbz2-dev libz-dev libicu-dev
    wget https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2
    tar -xvf boost_1_83_0.tar.bz2
    cd boost_1_83_0
    ./bootstrap.sh
    ./b2
    sudo ./b2 install --prefix=/usr/local
    cd
    rm -rf boost_1_83_0 boost_1_83_0.tar.bz2

    if ! grep -q "BOOST_ROOT" ~/.bashrc ; then
        echo '' >> ~/.bashrc
        echo 'export BOOST_ROOT=/usr/local' >> ~/.bashrc
        echo 'export PATH=$BOOST_ROOT/bin:$PATH' >> ~/.bashrc
        echo 'export LD_LIBRARY_PATH=$BOOST_ROOT/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
        echo 'export CPATH=$BOOST_ROOT/include:$CPATH' >> ~/.bashrc
        echo 'export LIBRARY_PATH=$BOOST_ROOT/lib:$LIBRARY_PATH' >> ~/.bashrc
        echo '' >> ~/.bashrc
        echo "Boost lines added to .bashrc"
    else
        echo "Boost lines already exist in .bashrc"
    fi

    source ~/.bashrc
}

# Function definition to install CMake
install_cmake () {
    echo "Installing CMake..."
    cd
    sudo apt-get install -y build-essential checkinstall zlib1g-dev libssl-dev
    wget https://github.com/Kitware/CMake/releases/download/v3.31.9/cmake-3.31.9.tar.gz
    tar -zxvf cmake-3.31.9.tar.gz
    cd cmake-3.31.9
    ./bootstrap
    make
    sudo make install
    cd
    rm -rf cmake-3.31.9 cmake-3.31.9.tar.gz
    cmake --version
}

# Function definition to install XSD
install_xsd () {
    echo "Installing XSD..."
    cd
    sudo apt-get install -y libxerces-c-dev xsdcxx
}

# Function definition to install OpenSim core
# Based on https://raw.githubusercontent.com/opensim-org/opensim-core/refs/heads/main/scripts/build/opensim-core-linux-build-script.sh
install_opensim_core () {
    echo "Installing OpenSim core..."
    # Ensure the script is run from its own directory 
    cd "$script_path" || { echo "Failed to change directory to script path"; exit 1; }
    sudo chmod +x opensim-core-linux-build-script.sh
    ./opensim-core-linux-build-script.sh -j 1 -s -d "Debug"
    echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bashrc
    source ~/.bashrc
}

# Function definition to install GLEW
install_glew () {
    echo "Installing GLEW..."
    cd
    sudo apt-get install -y libglew-dev
}

# Function definition to install Qt
install_qt () {
    echo "Installing Qt..."
    cd
    sudo apt-get install -y qt5-qmake qtbase5-dev
}

# Update and upgrade the system
sudo apt-get update && sudo apt-get upgrade -y

# Check for reboot requirements
if [ -f /var/run/reboot-required ]; then
    echo 'Reboot required. Please reboot your system and re-run the script.'
    exit 1
fi

script_path=$(dirname "$(realpath "$0")")
install_cmake
install_boost
install_xsd
install_glew
install_qt
install_opensim_core

# Move into the cloned core repository
cd "$(dirname "$script_path")" || { echo "Failed to change directory to repo path"; exit 1; }

# Create and enter the build directory
mkdir -p build
cd build

# Run CMake with GCC and necessary flags
cmake .. -DUSE_GUI=ON -DCOMPILE_PLUGIN=OFF -DOpenSim_DIR=~/opensim-core/cmake -Wno-dev -DCMAKE_CXX_FLAGS="-std=c++14 -w" -DCALIBRATION_REFLEX=ON -DCMAKE_BUILD_TYPE=$DEBUG_TYPE
cmake --build .

