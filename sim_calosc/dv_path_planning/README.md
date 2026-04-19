# PathPlanner

## Brief Description

The `PathPlanner` module is responsible for computing a trajectory for the bolide to follow, leveraging detected cones as input. It processes data from either a simulated cone detector or a SLAM system and generates a smooth path that adheres to the bolide's dynamics and geometric constraints. The computed path is then published for downstream use by the control module.

---
![PathPanningScheme (2)](https://github.com/user-attachments/assets/577abe29-e0f4-4414-9f7f-06a193a2255d)


---
## Communication with Other Modules

### Subscribed Topics

- **`/dv_cone_detector_fake/cones`** *(Conditionally Subscribed)*
  - **Type**: Custom message (`dv_interfaces::Cones`)
  - **Purpose**: Used when the `fake_cone_detector` parameter is set to `true`. Provides simulated cone detections for testing purposes.
  - **Callback**: `fakeConeCallback`

- **`/slam/cones`** *(Conditionally Subscribed)*
  - **Type**: `sensor_msgs::PointCloud2`
  - **Purpose**: Used when `fake_cone_detector` is `false`. Provides cone data from SLAM for path planning.
  - **Callback**: `coneCallback`

---

### Published Topics

- **`/path_planning/path`**
  - **Type**: `geometry_msgs::PoseArray`
  - **Purpose**: Publishes the planned path as a series of poses, with each pose representing a waypoint in the trajectory. The path coordinates are in the `"map"` frame.
  - **Publishing Method**: `publishPath`

---

## Parameters

### **`fake_cone_detector`**
- **Type**: `bool`
- **Purpose**: Selects the source of cone data:
  - `true`: Uses the simulated cone detector (`/dv_cone_detector_fake/cones`).
  - `false`: Uses SLAM for cone detection (`/slam/cones`).
- **Default Value**: Not explicitly set, but defaults to `false` if not provided.
- **Error Handling**: Logs an error if the parameter is not loaded.

---

## Key Algorithms

### Search Graph

The **search graph** is the core algorithm used in **PathPlanner** to discretize the space between cones and compute a path using graph-based pathfinding. The process includes the following steps:

#### 1. Initialize the Discretization Graph
- **Purpose**: Creates nodes between pairs of cones on the left and right chains.
- **Details**:
  - Nodes are added at the midpoints between cones that are closer than a specified maximum distance (`MAX_DISTANCE`).
  - The start and target positions are explicitly added as graph nodes.
- **Function**: `initializeDiscretizationGraph`

#### 2. Connect Reachable Nodes
- **Purpose**: Establishes bidirectional edges between nodes based on distance and proximity constraints.
- **Details**:
  - Nodes are connected if their distance is between `MIN_DISTANCE_BETWEEN_PATH_POINTS` and `MAX_DISTANCE_BETWEEN_PATH_POINTS`.
  - Connections must not increase the distance to the target.
- **Functions**:
  - `makeConnectedIfIsReachable`: Adds individual connections.
  - `connectAllReachableNodes`: Connects all nodes in the graph.

#### 3. Pathfinding with A*
- **Purpose**: Computes the optimal path from the start node to the target node.
- **Details**:
  - Uses a priority queue to explore nodes with the lowest cost-to-goal estimate (`f-value`).
  - `f-value` is calculated as:  
    `f-value = g-value (actual cost) + h-value (heuristic estimate of distance to target)`
  - Tracks visited nodes and costs using `cameFrom` and `costFromStartToNode` maps.
- **Function**: `computePath`

#### 4. Reconstruct the Path
- **Purpose**: Produces the final path by backtracking through the `cameFrom` map.
- **Function**: `reconstructPath`

---

## Key Functions

### `coneCallback`
- **Input**: `sensor_msgs::PointCloud2` containing cone positions from SLAM.
- **Purpose**: 
  - Processes SLAM cone data to plan a path.
  - Updates the bolide’s pose and runs the planning algorithm.
  - Publishes the path and visualization markers.

### `fakeConeCallback`
- **Input**: `dv_interfaces::Cones` containing simulated cone positions.
- **Purpose**: Same as `coneCallback`, but processes data from the fake cone detector.

### `poseCallback`
- **Purpose**: Updates the bolide’s current position and orientation using a transformation from the "map" frame to the "bolide_CoG" frame.
- **Details**:
  - Retrieves the transform using `tf2_ros::TransformListener`.
  - Extracts position and yaw angle.

### `publishPath`
- **Input**: A timestamp for the path header.
- **Purpose**: Converts the computed path into a `geometry_msgs::PoseArray` and publishes it on the `/path_planning/path` topic.

### `interpolatePath`
- **Purpose**: Smooths the path by adding intermediate points using spline interpolation.
- **Details**: Uses a cubic spline to generate points between consecutive waypoints in the path.

---

## Background

The **PathPlanner** module relies on the geometric relationships between cones and uses graph-based algorithms to determine a feasible path. The bolide's position and orientation are updated dynamically using transformations between frames.

---

## Dependencies

### ROS Packages:
- `geometry_msgs`
- `sensor_msgs`
- `tf2_ros`
- `dv_interfaces`
- `pcl_ros`

### Libraries:
- **PCL** (Point Cloud Library)
- **Spline Interpolation** (custom implementation)

---

## Running tests

```bash
catkin_make run_tests --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_LD_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="-lgcov"
```

## Running coverage
Make sure you have `coverage` directory under your package directory.

```bash
gcovr -r src build --filter '(src/driverless-software/dv_path_planning/src/.*)|(src/driverless-software/dv_path_planning/include/.*)' --html-details src/driverless-software/dv_path_planning/coverage/coverage.details.html
```
---

## Resources
- **" A * Pathfinding Algorithm"**, Amit Patel
- **" Spline Interpolation for Robotics"**, Paul Bourke
