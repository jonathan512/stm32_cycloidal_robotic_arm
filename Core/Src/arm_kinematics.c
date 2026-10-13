#include "arm_kinematics.h"

#include <math.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

static float wrap_deg(float deg)
{
  while (deg > 180.0f)
  {
    deg -= 360.0f;
  }
  while (deg < -180.0f)
  {
    deg += 360.0f;
  }
  return deg;
}

/* Radial reach and height (relative to the table) of the wrist and tip. */
static void planar_points(const arm_joints_t *j, float *wrist_r, float *wrist_z, float *tip_r, float *tip_z)
{
  float a = j->shoulder * DEG2RAD;
  float b = (j->shoulder + j->elbow) * DEG2RAD;
  float c = (j->shoulder + j->elbow + j->wrist) * DEG2RAD;

  *wrist_r = ARM_LINK1_MM * sinf(a) + ARM_LINK2_MM * sinf(b);
  *wrist_z = ARM_SHOULDER_Z_MM + ARM_LINK1_MM * cosf(a) + ARM_LINK2_MM * cosf(b);
  *tip_r = *wrist_r + ARM_POINTER_MM * sinf(c);
  *tip_z = *wrist_z + ARM_POINTER_MM * cosf(c);
}

arm_point_t arm_forward(const arm_joints_t *joints)
{
  arm_point_t p;
  float wr;
  float wz;
  float tr;
  float tz;
  float phi = joints->base * DEG2RAD;

  planar_points(joints, &wr, &wz, &tr, &tz);
  p.x = -tr * sinf(phi);
  p.y = tr * cosf(phi);
  p.z = tz;
  return p;
}

arm_ik_status_t arm_check_joints(const arm_joints_t *joints)
{
  float wr;
  float wz;
  float tr;
  float tz;

  if ((fabsf(joints->base) > ARM_BASE_LIMIT_DEG) ||
      (fabsf(joints->shoulder) > ARM_SHOULDER_LIMIT_DEG) ||
      (fabsf(joints->elbow) > ARM_ELBOW_LIMIT_DEG) ||
      (fabsf(joints->wrist) > ARM_WRIST_LIMIT_DEG))
  {
    return ARM_IK_JOINT_LIMIT;
  }

  planar_points(joints, &wr, &wz, &tr, &tz);
  if ((wz < ARM_WRIST_Z_MIN_MM) || (tz < ARM_TIP_Z_MIN_MM))
  {
    return ARM_IK_TOO_LOW;
  }

  return ARM_IK_OK;
}

arm_ik_status_t arm_inverse(const arm_point_t *tip, float prev_base_deg, arm_joints_t *out)
{
  const float l1 = ARM_LINK1_MM;
  const float l2 = ARM_LINK2_MM;
  const float pitch = ARM_POINTER_PITCH_DEG * DEG2RAD;
  float r = sqrtf((tip->x * tip->x) + (tip->y * tip->y));
  float base_deg;
  float wr;
  float wz;
  float d2;
  float cos_b;
  float beta;
  float alpha;
  arm_joints_t result;
  arm_ik_status_t status;

  if (r < ARM_TIP_R_MIN_MM)
  {
    return ARM_IK_TOO_CLOSE_TO_BASE;
  }
  if (tip->z < ARM_TIP_Z_MIN_MM)
  {
    return ARM_IK_TOO_LOW;
  }

  /* Choose the base angle equivalent to atan2(-x, y) that is nearest the previous one. */
  base_deg = atan2f(-tip->x, tip->y) * RAD2DEG;
  base_deg = prev_base_deg + wrap_deg(base_deg - prev_base_deg);

  /* Wrist position in the arm plane, relative to the shoulder axis. */
  wr = r - ARM_POINTER_MM * sinf(pitch);
  wz = tip->z - ARM_SHOULDER_Z_MM - ARM_POINTER_MM * cosf(pitch);

  d2 = (wr * wr) + (wz * wz);
  cos_b = (d2 - (l1 * l1) - (l2 * l2)) / (2.0f * l1 * l2);
  if (cos_b > 1.0f)
  {
    return ARM_IK_OUT_OF_REACH;
  }
  if (cos_b < -1.0f)
  {
    return ARM_IK_TOO_CLOSE;
  }

  /* Elbow-up solution: elbow angle is non-negative. */
  beta = acosf(cos_b);
  alpha = atan2f(wr, wz) - atan2f(l2 * sinf(beta), l1 + (l2 * cosf(beta)));

  result.base = base_deg;
  result.shoulder = alpha * RAD2DEG;
  result.elbow = beta * RAD2DEG;
  result.wrist = (pitch - alpha - beta) * RAD2DEG;

  status = arm_check_joints(&result);
  if (status != ARM_IK_OK)
  {
    return status;
  }

  *out = result;
  return ARM_IK_OK;
}

const char *arm_ik_status_text(arm_ik_status_t status)
{
  switch (status)
  {
    case ARM_IK_OK:
      return "ok";
    case ARM_IK_TOO_CLOSE_TO_BASE:
      return "too close to base axis";
    case ARM_IK_TOO_LOW:
      return "too low (table)";
    case ARM_IK_OUT_OF_REACH:
      return "out of reach";
    case ARM_IK_TOO_CLOSE:
      return "too close to shoulder";
    case ARM_IK_JOINT_LIMIT:
      return "joint limit";
    default:
      return "error";
  }
}
