 # Simple ROS

This is a rapid introduction to working with ROS2. For further information about the code examples or ROS features used, each folder contains both commented code and an associated readme similar to the one you are reading now.

## Table of Contents
1. [Prerequisites](#Prerequisites)
2. [Creating a project](#creating-a-project)
3. [Project structure](#project-structure)
4. [Running an example](#running-an-example)

## Prerequisites

To start you'll need to install ROS and the colcon build tool.

* [ROS install Windows](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html)
* [ROS install Ubuntu](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)
* [Colcon - Used for building and creating packages](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon)

We can also install visual studio code while we are at it, to ensure we are all using the same development environment. I am open to change on the IDE we use, but I have a lot of experience in vscode.

* [Visual Studio Code](https://code.visualstudio.com/)

Additionally, here are a few required extensions you should install as well. These can be obtained by following the links or by typing in the extension name into visual studio code extensions (`ctrl + shift + x` when in vscode) or by clicking the extensions button on the left hand side menu.

* [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
* [Remote -SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
* [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
* [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

Furthermore, here are a few helpful extensions, I would recommend.

* [Better C++ Syntax](https://marketplace.visualstudio.com/items?itemName=jeff-hykin.better-cpp-syntax)
* [GitLens — Git supercharged](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens)
* [Panda Theme](https://marketplace.visualstudio.com/items?itemName=tinkertrain.theme-panda)

## Creating a Project

Remember to 'source' you're workspace (folder) before you try any of these ROS related commands. The sourcing command will look something like this.

> Windows - `call C:\dev\ros2_foxy\local_setup.bat`

> Linux - `. ~/ros2_foxy/ros2-linux/setup.bash`

We will following the below tutorial for creating a simple project using Linux. (The steps maybe slightly different for windows)

[Ros package creator](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html#create-a-package)   

1. `ros2 pkg create --build-type ament_cmake --node-name simple_node simple_package`
2. `cd ./simple_package`
3. `colcon build`
4. `. install/setup.bash`
5. `ros2 run simple_package simple_node`

The first command follows this format:

> `ros2 pkg create --build-type ament_cmake --node-name [NODE NAME] [PACKAGE NAME]`

Where you can fill in the node and package names. The parameter `--node-name`, creates an additional default 'hello world' style cpp, so we can build the program without having to complete an extra step. This parameter can be removed in later package creation processes. 

The rest of these commands will be explained in greater detail [below](#running-an-example), as this is just meant as a starting point.

## Project structure 

If no errors occur, the resulting file structure **after step one** should be:

```
package_name
│   CMakeLists.txt
│   package.xml
│
└───src
│   │   node_name.cpp
│   
└───include
```

To start off with the easy ones, the source (src) and include folders should contain your source files (.c/.cpp/.py) and header files (.h). The only current source file (`node_name.cpp`), will contain a simple hello world program, as mentioned earlier. 

I recommend you do not investigate the other 2 files (CMakelists.txt/package.xml) too far. I would also recommend that you skip the next  sub-section and revisit it at a later time, once you are more confident with ROS and C++.

The final file structure (after step 3) should be:

```
package_name
│   CMakeLists.txt
│   package.xml
│
└───build
│   │   ...
│
└───include
│   │   
│   
└───install
│   │   ...
│   
└───log
│   │   ...
│  
└───src
│   │   ...
│
```

The log folder will contain logs of the build process, if you would like to see a detailed log of how ROS is building your package look under the folder `latest_build` and at the files `events.log` and `logger_all.log`.

The build folder will be where your project actaully exists as an executable file. The surrounding files and folders speed up rebuilding, and assist ROS in running your project. These files should not be touched.

The install folder is where all files required for installing your package to ROS are located. Most of these files are simple script moving files and setting ROS environment variables. These files should also not be touched.

### Build configuration

To begin I will break down the simpler of the two files, package.xml. XML is a markup langauge, similar to that of HTML. The structure of most markup langauges follow a **<property\>value</property\>** arrangement. To break it down further I will analyse the first few lines of the default XML file created earlier.

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
```

These two lines should be ignored as they are just setting the XML standard used in the document, and are therefore not relevant.

```
<package format="3">
```

Additionally, this line should be ignored as it is configuring the ROS package langauge format. If you really want to waste your time read [this](https://www.ros.org/reps/rep-0149.html). 

```
<name>simple_package</name>
```

As hard as it sounds, this is the package name. This will match the folder name. To change the package name you cannot just change this value. I will show how to do this in a later section.

```
<version>0.0.0</version>
```

The version number of your package can be set here. These numbers count from 0 and are increased by one each time a change on its level is made. The structure of these version numbers generally follows this format:

> MAJOR VERSION NUMBER . MINOR VERSION NUMBER . BUILD VERSION

Here is a guide for what the numbers mean:

>Major version - A complete rework of the project. \
>Minor version - A rework of a feature or component of the project. \
>Build version - A small fix of update to a feature of the project.

When a number of incremental improvements have been made to a feature, it is generally a good idea to move to the next minor version. For the sack of our University projects, this can be ignored as we will handle version control entirely in git.

You can probably pick up on the package.XML style and should be able understand the syntax behind the file. [This site](http://wiki.ros.org/catkin/package.xml) should be able to fill in an questions you have.

---

CMakeLists.txt is unfortunately quite complex if you do not understand make files and the C++ build process. For the sake of not losing my mind explaining the entire file, I will keep it brief and point you in the direction of somewhere you can learn it on your own. 

To change the package name, you must change the name property in `package.xml` and the line `project(simple_package)` both to the new package name. Although not required, it is generally good practice to also change the folder (workspace) name.

To change the node name you must change the following 3 lines:

```
add_executable([NEW NODE NAME] src/simple_node.cpp)

target_include_directories([NEW NODE NAME] PUBLIC $<BUILD_INTERFACE: {CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(TARGETS [NEW NODE NAME]
  DESTINATION lib/${PROJECT_NAME})
```

## Running an Example

To start running an example we first need to enter the directory of the example. Ensure you have sourced the terminal first. ([provided above](#creating-a-project))

> `cd ./simple_topic`

Next the project can be built using the command.

> `colcon build`

If you skipped or missed entering the directory, all projects will be built and workspace will be built instead.

Then the project can be added to the ROS environment path. Essentially think of this command as giving ROS knowledge of the project you've just made.

> `. install/setup.bash`

Finally, we can run the project. The package name can be found by looking at the folder name (for this repository) and the node name can be found by looking at the executable name in the build folder and into the package folder. Or by looking at the `CMakeLists.txt` for the executable name.

> `ros2 run [PACKAGE NAME] [NODE NAME]`