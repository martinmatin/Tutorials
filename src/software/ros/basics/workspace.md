# Workspaces
The first thing you will want to do before you write code is create an environnement for your project. This environnement is called *Workspace*. The Workspace is the root folder containing subfolders and files essential to run your project.
These files will be generated using the **catkin_init_workspace** command.

Let's get into it!
<br><br>

* Choose a directory for your workspace, let's use 'catkin_ws'
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
This will create a **CMakeLists.txt** file in your **/src** folder.
<br><br>

* Then we will make the project
```
cd ~/catkin_ws
catkin_make
```
This will create two subfolders, **build** and **devel**. The **build** folder is none of our interest in most of the part. The **devel** folder contains a number of files and directories, the most interesting of which are the setup files. Running these configures your system to use this workspace, and the code that’s (going to be) contained inside it.
<br><br>

* Configure your machine to use this Workspace
```
source devel/setup.bash
```


That's it, you've got your workspace up and running!
