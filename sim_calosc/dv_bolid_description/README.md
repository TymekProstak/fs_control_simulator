# dv_bolid_description

## Brief Description

Package is used primarily for broadcasting the transformations between frames of reference on the car. Transformations can be checked on `/tf_static` and `/tf` topics and seen in Rviz.

##  Usage

Launch the core functionality with:

```bash
roslaunch dv_bolid_description description.launch
```

### Launch files

- **description.launch** - launches the broadcast of transformations

  #### Arguments
    - **`use_sim`** _(Default: `False`)_ - launch the simulator model

### Config files

Frame tree is specified in urdf files, seperately for real world (*`LEM_bolid_model.urdf.xacro`*) and simulation (*`fsds_bolid_model.urdf.xacro`*).

### Related and useful

Prints specified transform to the console
```bash
rosrun tf tf_echo <source frame> <target frame>
```

Plots a graph of the whole tf tree
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

