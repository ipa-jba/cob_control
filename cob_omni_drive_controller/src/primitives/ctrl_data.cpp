#include <angles/angles.h>
#include <cob_omni_drive_controller/primitives/ctrl_data.h>
#include <cob_omni_drive_controller/utils/math_sup.h>
#include <math.h>

namespace cob_omni_drive_controller
{
void CtrlData::pickConfiguration(double steer_target, double drive_target)
{
  double st2 = steer_target + M_PI;
  MathSup::normalizePi(st2);

  if ((MathSup::getWeightedDelta(state_.dAngGearSteerRad, m_dAngGearSteerTargetRad, steer_target) <=
       MathSup::getWeightedDelta(state_.dAngGearSteerRad, m_dAngGearSteerTargetRad, st2)) &&
      geom_.limits.positionInLimits(steer_target))
  {
    // Target1 is "optimal"
    m_dVelGearDriveTargetRadS = drive_target;
    m_dAngGearSteerTargetRad = steer_target;
  }
  else
  {
    // Target2 is "optimal"
    m_dVelGearDriveTargetRadS = -drive_target;
    m_dAngGearSteerTargetRad = st2;
  }
}
void CtrlData::setTarget(const PlatformState& plt_state)
{
  // check if zero movement commanded -> keep orientation of wheels, set wheel velocity to zero
  if ((plt_state.dVelLongMMS == 0) && (plt_state.dVelLatMMS == 0) && (plt_state.dRotRobRadS == 0))
  {
    m_dVelGearDriveTargetRadS = 0.0;
    m_dAngGearSteerTargetRad = state_.dAngGearSteerRad;
    return;
  }

  // calculate velocity and direction of single wheel motion
  // Translational Portion
  double dtempAxVelXRobMMS = plt_state.dVelLongMMS;
  double dtempAxVelYRobMMS = plt_state.dVelLatMMS;
  // Rotational Portion
  dtempAxVelXRobMMS += plt_state.dRotRobRadS * m_dExWheelDistMM * -sin(m_dExWheelAngRad);
  dtempAxVelYRobMMS += plt_state.dRotRobRadS * m_dExWheelDistMM * cos(m_dExWheelAngRad);

  // calculate resulting steering angle
  // Wheel has to move in direction of resulting velocity vector of steering axis
  double dAngGearSteerTarget1Rad = MathSup::atan4quad(dtempAxVelYRobMMS, dtempAxVelXRobMMS);

  // calculate absolute value of rotational rate of driving wheels in rad/s
  double dVelGearDriveTarget1RadS =
      sqrt((dtempAxVelXRobMMS * dtempAxVelXRobMMS) + (dtempAxVelYRobMMS * dtempAxVelYRobMMS)) / geom_.dRadiusWheelMM;

  pickConfiguration(dAngGearSteerTarget1Rad, dVelGearDriveTarget1RadS);
}

void CtrlData::calcControlStep(WheelCommand& command, double dCmdRateS, bool reset)
{
  if (reset)
  {
    this->reset();
    command.dVelGearDriveRadS = 0.0;
    command.dVelGearSteerRadS = 0.0;
    command.dAngGearSteerRad = state_.dAngGearSteerRad;
    command.dAngGearSteerRadDelta = 0.0;
    return;
  }

  // Normalize Actual Wheel Position before calculation
  double dCurrentPosWheelRAD = angles::normalize_angle(state_.dAngGearSteerRad);
  command.dAngGearSteerRadDelta = angles::normalize_angle(m_dAngGearSteerTargetRad - dCurrentPosWheelRAD);

  // set outputs
  command.dVelGearDriveRadS = MathSup::limitValue(m_dVelGearDriveTargetRadS + m_dAngGearSteerTargetRad * dFactorVel,
                                                  params_.dMaxDriveRateRadpS);

  // provisorial --> skip interpolation and always take Target
  command.dAngGearSteerRad = m_dAngGearSteerTargetRad;
}

void CtrlData::reset()
{
}
}  // namespace cob_omni_drive_controller
