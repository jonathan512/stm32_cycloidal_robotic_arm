#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Arm geometry and kinematics (units: mm and degrees).
 *
 * World frame: origin on the table at the base axis, Z up.
 *   Base angle 0 -> arm plane is the Y-Z plane and the arm reaches toward +Y.
 *   Positive base angle rotates counter-clockwise seen from above (+Y toward -X).
 *
 * Joint angles (all 0 deg at the "home" pose = every link pointing straight up):
 *   base     : rotation about Z
 *   shoulder : link 1 angle from vertical, positive leans toward the reach direction
 *   elbow    : link 2 angle relative to link 1, positive bends the same way
 *   wrist    : pointer angle relative to link 2, positive bends the same way
 *
 * Pointer pitch from vertical = shoulder + elbow + wrist (90 deg = horizontal).
 */

#define ARM_LINK1_MM          100.0f
#define ARM_LINK2_MM          100.0f
#define ARM_POINTER_MM        100.0f
#define ARM_SHOULDER_Z_MM     121.0f

#define ARM_BASE_LIMIT_DEG       360.0f
#define ARM_SHOULDER_LIMIT_DEG    90.0f
#define ARM_ELBOW_LIMIT_DEG      135.0f
#define ARM_WRIST_LIMIT_DEG      135.0f

/* Workspace guards for the tip (pointer held horizontal) */
#define ARM_TIP_Z_MIN_MM          20.0f
#define ARM_WRIST_Z_MIN_MM        20.0f
#define ARM_TIP_R_MIN_MM          50.0f

#define ARM_POINTER_PITCH_DEG     90.0f

typedef struct
{
  float base;
  float shoulder;
  float elbow;
  float wrist;
} arm_joints_t;

typedef struct
{
  float x;
  float y;
  float z;
} arm_point_t;

typedef enum
{
  ARM_IK_OK = 0,
  ARM_IK_TOO_CLOSE_TO_BASE,
  ARM_IK_TOO_LOW,
  ARM_IK_OUT_OF_REACH,
  ARM_IK_TOO_CLOSE,
  ARM_IK_JOINT_LIMIT
} arm_ik_status_t;

/* Tip position for the given joint angles (degrees). */
arm_point_t arm_forward(const arm_joints_t *joints);

/*
 * Inverse kinematics with the pointer held at ARM_POINTER_PITCH_DEG.
 * prev_base_deg keeps the base angle continuous (no +/-180 deg flips).
 * Joint angles are written to out only when ARM_IK_OK is returned.
 */
arm_ik_status_t arm_inverse(const arm_point_t *tip, float prev_base_deg, arm_joints_t *out);

/* Checks joint limits and keeps the elbow/wrist/tip above the table. */
arm_ik_status_t arm_check_joints(const arm_joints_t *joints);

const char *arm_ik_status_text(arm_ik_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */
