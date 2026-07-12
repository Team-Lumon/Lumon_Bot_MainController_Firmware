#ifndef IK_WRAPPER_H
#define IK_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Matches the MotorCommand struct in IK.cpp
typedef struct {
  float L_total;
  float L_dot;
  float tau;
  bool feasible;
} MotorCommand_t;

// Vector3 structure for C interface
typedef struct {
  float x;
  float y;
  float z;
} Vector3_t;

// Initializes the pulley and end-effector geometry
void initGeometry(void);

// Computes target lengths, velocities, and tensions for a given time t
void computeFrameTargetsWrapper(float t, const Vector3_t* r_start, const Vector3_t* r_end, 
                                MotorCommand_t* commands, Vector3_t* r_out, Vector3_t* r_dot_out);

#ifdef __cplusplus
}
#endif

#endif // IK_WRAPPER_H
