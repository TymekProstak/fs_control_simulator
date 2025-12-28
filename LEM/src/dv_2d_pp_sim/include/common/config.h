#pragma once

// distance is in meters, angles in radians

// path planner
#define MAX_DISTANCE_BETWEEN_PATH_POINTS 4.0
#define MIN_DISTANCE_BETWEEN_PATH_POINTS 1.1

// path interpolation
#define MAX_DISTANCE_BETWEEN_INTERPOLATED_PATH_POINTS 1.5

// cone chain
#define CAPACITY_TO_RESERVE 512
#define MAX_CONE_COST 15.0
#define MAX_DISTANCE_TO_NEXT_CONE_IN_CHAIN 10 // sensor range + 3
#define MIN_DISTANCE_TO_NEXT_CONE_IN_CHAIN 0.5
#define MAX_ANGLE_TO_NEXT_CONE_IN_CHAIN 1.1 // in radians, 1.1rad=63deg
#define MAX_DISTANCE_BETWEEN_INTERPOLATED_CONES 1.2

// bolide desc
#define BOLIDE_WIDTH 2.6           // distance between source cones
#define BOLIDE_LENGTH_TILL_END 0.5 // distance between base_link and the middle source cone

// geometry
#define EPS 0.0001 // epsilon