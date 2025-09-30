# Introduction

inno-lidar-sdk is the software development kit to help application developers easily interact with lidar. the development documentation([HOW_TO_USE_CLIENT_SDK.md](./docs/HOW_TO_USE_CLIENT_SDK.md)) and sample code([demo.cpp](./apps/example/demo.cpp)) are provided to allow developers to get quickly started. Besides, there are some useful execution files and scripts in ```apps``` directory. For detailed information, please refer to the corresponding documents in each directory.

# Supported Lidar

+ Falcon-K1
+ Falcon-K2
+ Falcon-k24
+ Robin-W
+ Robin-ELITE(Robin-E1X)
+ Robin-E2
+ HB-D1


# Supported Platforms

+ MacOs
+ Windows
+ linux
+ qnx


# Quick Compile

## for Linux and MacOS

- Prerequisites
  + gcc
  + cmake
```
cd build && ./build_unix.sh
```

## for windows

- Prerequisites
  + Visual studio
  + cmake
+ Generate the Visual Studio Project

```
mkdir build_win
cd build_win
cmake ..
```
- Open the Visual Studio Solution and compile


# Quick Start
inno-lidar-sdk offers two demo programs in ./apps/example directory

-   ```demo.cpp```

   connects to online lidar, and callback by xyz frame.

-   ```sphere2xyz.cpp```

   connects to online lidar, and callback by sphere packet, upper-layer implements frame assembly logic and coordinate conversion.


# Directory Structure

+ ```apps```: Source files to generate applications
  + ```example```: Demo about how to get pointcloud. Please press *[here](./docs/demo.md)* to see detailed information.
  + ```tools``` All kinds of scripts and excutive file
    + ```check_net``` Tool for network check. Please press *[here](./docs/check_net.md)* to see detailed information.
    + ```get_pcd```  Tool for obtaining pointcloud and converting file format. Please press *[here](./docs/get_pcd.md)* to see detailed information.
    + ```innovusion_wireshark_plugin``` Lua script for wireshark. Please press *[here](./docs/innovusion_lua.md)* to see detailed information.
    + ```innovusion_lidar_util``` lidar configuration tools.
+ ```docs``` Documents help compile and use client SDK
+ ```build``` All kinds of build script
+ ```lib``` Library files
+ ```CMakeLists.txt``` Compile script by CMake
+ ```Makefile``` Compile script by Make
+ ```README.md``` Introduction of Client SDK
+ ```src``` Source files to generate library files
  + ```sdk_client``` Source files to build sdk_client library
  + ```sdk_common``` Source files to build sdk_common library
  + ```utils``` Source files to build utility library

```shell
├── apps
│   ├── example
│   └── tools
│       ├── check_net
│       ├── get_pcd
│       ├── innovusion_wireshark_plugin
│       ├── lidar_util
├── build
├── docs
├── CMakeLists.txt
├── lib
├── Makefile
├── README.md
└── src
    ├── sdk_client
    ├── sdk_common
    └── utils
```
