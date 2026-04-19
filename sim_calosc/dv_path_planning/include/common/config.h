#pragma once

//#define PERFORM_PARAMETER_OPTIMIZATION

// distance is in meters, angles in radians

// path planner
#define MAX_DISTANCE_BETWEEN_PATH_POINTS 6.0
#define MIN_DISTANCE_BETWEEN_PATH_POINTS 1.1

// path interpolation
#define MAX_DISTANCE_BETWEEN_INTERPOLATED_PATH_POINTS 1.5

// bolide desc
#define BOLIDE_WIDTH 2.6           // distance between source cones
#define BOLIDE_LENGTH_TILL_END 0.5 // distance between base_link and the middle source cone

// geometry
#define EPS 0.0001 // epsilon

// Path-Planning algo
#define TRACK_STEP_SIZE 0.2 // Percent of track width to take a forward track step, too big can skip cones, too small is slow (think of it like precision)
#define POSITION_PROXIMITY_THRESHOLD 2.0
#define PATH_PLANNING_MAX_ITERATIONS 1000
#define LAP_COUNTING_ANGLE_THRESHOLD 0.92 // dot product (0,1)
#define LAP_COUNTING_FINISH_MIN_DIST 3

#define NUMBER_OF_PATH_POINTS 1000 // Number of points in the path
#define INTERPOLATION_TYPE 0 // 0 - No Interp. 1 - Linear, 2 - Akima, 3 - Cubic, 4 - Polynomial

// Gauss-Newton
#define GN_MAX_ITER 10
#define GN_PRECISION 1e-2
#define GN_POLYNOMIAL_DEGREE 3
#define GN_DISTANCE_BETWEEN_POINTS 0.5
