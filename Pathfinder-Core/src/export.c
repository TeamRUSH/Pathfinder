#include <stdio.h>
#include <emscripten/emscripten.h>
#include "pathfinder.h"


#ifdef __cplusplus
extern "C" {
#endif

void s2a(Segment *trajectory, int trajLen, double *buf) {
  int i;
  for(i = 0; i < trajLen; i++) {
    buf[8*i] = trajectory[i].dt;
    buf[8*i+1] = trajectory[i].x;
    buf[8*i+2] = trajectory[i].y;
    buf[8*i+3] = trajectory[i].position;
    buf[8*i+4] = trajectory[i].velocity;
    buf[8*i+5] = trajectory[i].acceleration;
    buf[8*i+6] = trajectory[i].jerk;
    buf[8*i+7] = trajectory[i].heading;
  }
}

void merge(double *buf, 
            int buflen, 
            double *arr1,
            double *arr2,
            double *arr3, 
            int arrLen) {
  int i;
  for(i = 0; i < arrLen; i++) {
    buf[i] = arr1[i];
  }

  for(i = 0; i < arrLen; i++) {
    buf[i + arrLen] = arr2[i];
  }

  for(i = 0; i < arrLen; i++) {
    buf[i + 2 *arrLen] = arr3[i];
  }
}

long EMSCRIPTEN_KEEPALIVE path_length(double *wps,
                                        int wp_len,
                                        int fit,
                                        int sample_count,
                                        double track_width,
                                        double dt,
                                        double max_velocity,
                                        double max_acceleration,
                                        double max_jerk) {

    int POINT_LENGTH = wp_len / 3;

    Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

    int i;
    for(i = 0; i < POINT_LENGTH; i++) {
        Waypoint pt = { wps[3*i], wps[3*i+1], wps[3*i+2] };
        points[i] = pt;
    }

    TrajectoryCandidate candidate;
    pathfinder_prepare(points, 
                        POINT_LENGTH, 
                        fit == 1 ? FIT_HERMITE_QUINTIC : FIT_HERMITE_CUBIC, 
                        sample_count, 
                        dt, 
                        max_velocity, 
                        max_acceleration, 
                        max_jerk, 
                        &candidate);
    
    return candidate.length;
}

double* EMSCRIPTEN_KEEPALIVE find_path(double *wps, 
                                        int wp_len, 
                                        int fit, 
                                        int sample_count, 
                                        double track_width, 
                                        double dt, 
                                        double max_velocity, 
                                        double max_acceleration,
                                        double max_jerk) {

    int POINT_LENGTH = wp_len / 3;

    Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

    int i;
    for(i = 0; i < POINT_LENGTH; i++) {
        Waypoint pt = { wps[3*i], wps[3*i+1], wps[3*i+2] };
        points[i] = pt;
    }

    TrajectoryCandidate candidate;
    pathfinder_prepare(points, 
                        POINT_LENGTH, 
                        fit == 1 ? FIT_HERMITE_QUINTIC : FIT_HERMITE_CUBIC, 
                        sample_count, 
                        dt, 
                        max_velocity, 
                        max_acceleration, 
                        max_jerk, 
                        &candidate);

    int length = candidate.length;
    Segment *trajectory = malloc(length * sizeof(Segment));

    pathfinder_generate(&candidate, trajectory);

    Segment *leftTrajectory = malloc(length * sizeof(Segment));
    Segment *rightTrajectory = malloc(length * sizeof(Segment));

    pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, track_width);
    
    int segArrLen = length * 8;
    double *segArr = malloc(sizeof(double) * segArrLen); // 8 fields per segment
    double *lSegArr = malloc(sizeof(double) * segArrLen); // 8 fields per segment
    double *rSegArr = malloc(sizeof(double) * segArrLen); // 8 fields per segment

    s2a(trajectory, length, segArr);
    s2a(leftTrajectory, length, lSegArr);
    s2a(rightTrajectory, length, rSegArr);

    double *ret = malloc(sizeof(double) * segArrLen * 3);

    merge(ret, 
            segArrLen * 3, 
            segArr, 
            lSegArr, 
            rSegArr,
            segArrLen);

    return ret;
}


#ifdef __cplusplus
}
#endif