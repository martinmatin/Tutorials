# Launch files

Launch files are very common in ROS to both users and developers. They provide a convenient way to start up multiple nodes and a master, as well as other initialization requirements such as setting parameters.

## Roslaunch
`roslaunch` is used to open launch files. This can be done by either specifying the package the launch files are contained in followed by the name of the launch file, or by specifying the file path to the launch file.

```bash
roslaunch package_name launch_file arg1:=value arg2:=value
````

>roslaunch will also start `roscore` if no master has been set. Pushing `Ctrl-C` in a terminal with a launch file running will close all nodes that were started with that launch files.


## Writing a .launch files

Launch files are of the format .launch and use a specific XML format. They can be placed anywhere within a package directory, but it is common to make a directory named **Launch** inside the workspace directory to organize all your launch files. The contents of a launch file must be contained between a pair of launch tags
```xml
<launch><!-- Content here --></launch>
```
To actually start a node, the `<node>` tags are used, the **pkg**, **type** and **name** argument are required.

```xml
<node pkg="..." type="..." name="..." respawn=true/>
```
* `pkg, type and name`: The argument **pkg** points to the package associated with the node that is to be launched, while **type** refers to the name of the node executable file.

* `respawn or required`: However optional, it’s common to either have a respawn argument or a required argument, but not both. If **respawn=true**, then this particular node will be restarted if for some reason it closed. **required=true** will do the opposite, that is, it will shut down all the nodes associated with a launch file if this particular node comes down. There are other optional argument available on the ROS wiki.

* `arg`: Sometimes it is necessary to use a local variable in launch files. This can be done using

```xml
<arg name="..." value="...">
```

