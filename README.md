 # Simple ROS Tutorial

This is a rapid introduction to working with ROS.

## Table of Contents
1. [Prerequisites](#Prerequisites)
2. [Creating a project](#creating-a-project)

## Prerequisites

To start you'll need to install ROS and the colcon build tool.

* [ROS install Windows](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html)
* [ROS install Ubuntu](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)
* [Colcon - Used for building and creating](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon)

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

>windows - `call C:\dev\ros2_foxy\local_setup.bat`

>linux - `. ~/ros2_foxy/ros2-linux/setup.bash`

We will following the below tutorial for creating a simple project using Linux. (The steps maybe slightly different for windows)

[Ros package creator](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html#create-a-package)   

1. `ros2 pkg create --build-type ament_cmake --node-name simple_node simple_package`
2. `cd ./simple_package`
3. `colcon build`
4. `. install/setup.bash`
5. `ros2 run simple_package simple_node`