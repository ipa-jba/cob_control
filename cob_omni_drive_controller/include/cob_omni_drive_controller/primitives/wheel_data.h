#pragma once
#include <cob_omni_drive_controller/types.h>
namespace cob_omni_drive_controller
{
struct WheelData
{
  WheelGeom geom_;

  /** Factor between steering motion and steering induced motion of drive wheels
   *  subtract from Drive-Wheel Vel to get effective Drive Velocity (Direct Kinematics)
   *  add to Drive-Wheel Vel (Inverse Kinematics) to account for coupling when commanding velos
   */
  double dFactorVel;

  WheelState state_;

  /** Exact Position of the Wheels' itself
   *  in cartesian (X/Y) and polar (Dist/Ang) coordinates
   *  relative to robot coordinate System
   */
  double m_dExWheelXPosMM;
  double m_dExWheelYPosMM;
  double m_dExWheelDistMM;
  double m_dExWheelAngRad;

  double m_dVelWheelMMS;

  void updateState(const WheelState& state);
  double getVelX() const;
  double getVelY() const;

  static double mergeRotRobRadS(const WheelData& wheel1, const WheelData& wheel2);

  WheelData(const WheelGeom& geom)
    : geom_(geom), dFactorVel(-geom_.dSteerDriveCoupling + geom_.dDistSteerAxisToDriveWheelMM / geom_.dRadiusWheelMM)
  {
    updateState(WheelState());
  }
};
}  // namespace cob_omni_drive_controller
