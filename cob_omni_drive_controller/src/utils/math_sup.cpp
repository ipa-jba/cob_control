
#include <angles/angles.h>
#include <math.h>
#include "cob_omni_drive_controller/utils/math_sup.h"

namespace MathSup
{
void normalizePi(double& val)
{
  val = angles::normalize_angle(val);
}

double atan4quad(double y, double x)
{
  if ((x == 0.0) && (y == 0.0))
    return 0;  // special case (?)
  return atan2(y, x);
}

double getWeightedDelta(double current_position, double old_target, double new_target)
{
  // current_position =  angles::normalize_angle(current_position); not needed if current Pos is alread normalized

  // Calculate differences between current config to possible set-points
  double dtempDeltaPhi1RAD = angles::normalize_angle(new_target - current_position);
  double dtempDeltaPhiCmd1RAD = angles::normalize_angle(new_target - old_target);

  // determine optimal setpoint value
  // 1st which set point is closest to current cinfog
  //     but: avoid permanent switching (if next target is about PI/2 from current config)
  // 2nd which set point is closest to last set point
  // "fitness criteria" to choose optimal set point:
  // calculate accumulted (+ weighted) difference between targets, current config. and last command
  return 0.6 * fabs(dtempDeltaPhi1RAD) + 0.4 * fabs(dtempDeltaPhiCmd1RAD);
}

double limitValue(double value, double limit)
{
  if (limit != 0)
  {
    if (value > limit)
    {
      value = limit;
    }
    else if (value < -limit)
    {
      value = -limit;
    }
  }
  return value;
}

}  // namespace MathSup
