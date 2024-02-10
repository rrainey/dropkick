#include <cstdint>
#include <math.h>

#ifdef POSE_GLOBALS_DEFINITIONS
#define _POSE_EXTERN 
#else
#define _POSE_EXTERN extern
#endif

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
_POSE_EXTERN float pi;
_POSE_EXTERN float GyroMeasError;   // gyroscope measurement error in rads/s (start at 40 deg/s)
_POSE_EXTERN float GyroMeasDrift;   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
_POSE_EXTERN float beta;   // compute beta
_POSE_EXTERN float zeta;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
_POSE_EXTERN uint32_t delt_t;                      // used to control display output rate
_POSE_EXTERN uint32_t sumCount;                    // used to control display output rate
_POSE_EXTERN float pitch, yaw, roll;                   // absolute orientation
_POSE_EXTERN float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
_POSE_EXTERN float deltat, sum;          // integration interval for both filter schemes
_POSE_EXTERN uint32_t lastUpdate, firstUpdate; // used to calculate integration interval
_POSE_EXTERN uint32_t Now;                         // used to calculate integration interval
_POSE_EXTERN float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
_POSE_EXTERN float q[4];    // vector to hold quaternion
_POSE_EXTERN float eInt[3];       // vector to hold integral error for Mahony method