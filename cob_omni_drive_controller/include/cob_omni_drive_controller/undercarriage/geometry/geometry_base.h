#pragma once
#include <stdexcept>
#include <vector>
#include "cob_omni_drive_controller/types.h"
#include "cob_omni_drive_controller/primitives/wheel_data.h"

namespace cob_omni_drive_controller
{
class UndercarriageGeomBase
{
public:
  // Get result of direct kinematics
  virtual void calcDirect(PlatformState& state) const = 0;

  // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
  virtual void updateWheelStates(const std::vector<WheelState>& states) = 0;

  virtual ~UndercarriageGeomBase()
  {
  }

protected:
  template <typename V>
  static void updateWheelStates(V& wheels, const std::vector<WheelState>& states)
  {
    if (wheels.size() != states.size())
      throw std::length_error("number of states does not match number of wheels");

    for (size_t i = 0; i < wheels.size(); ++i)
    {
      wheels[i]->updateState(states[i]);
    }
  }

  template <typename V>
  static void calcDirect(PlatformState& state, const V& wheels)
  {
    double dtempRotRobRADPS = 0;  // Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
    double dtempVelXRobMMS = 0;   // Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
    double dtempVelYRobMMS = 0;   // Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)

    // calculate rotational rate of robot and current "virtual" axis between all wheels
    for (int i = 0; i < wheels.size(); i++)
    {
      const WheelData& wheel = *wheels[i];
      const WheelData& other_wheel = *wheels[(i + 1) % wheels.size()];

      dtempRotRobRADPS += WheelData::mergeRotRobRadS(wheel, other_wheel);
      dtempVelXRobMMS += wheel.getVelX();
      dtempVelYRobMMS += wheel.getVelY();
    }

    // assign rotational velocities for output
    state.dRotRobRadS = dtempRotRobRADPS / wheels.size();

    // assign linear velocity of robot for output
    state.dVelLongMMS = dtempVelXRobMMS / wheels.size();
    state.dVelLatMMS = dtempVelYRobMMS / wheels.size();
  }
};

}  // namespace cob_omni_drive_controller
