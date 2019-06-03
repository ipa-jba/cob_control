#include <cob_omni_drive_controller/primitives/wheel_data.h>
#include <cob_omni_drive_controller/utils/math_sup.h>
#include <math.h>

namespace cob_omni_drive_controller
{
void WheelData::updateState(const WheelState& state)
{
  state_ = state;

  // calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
  m_dExWheelXPosMM = geom_.dWheelXPosMM + geom_.dDistSteerAxisToDriveWheelMM * sin(state_.dAngGearSteerRad);
  m_dExWheelYPosMM = geom_.dWheelYPosMM - geom_.dDistSteerAxisToDriveWheelMM * cos(state_.dAngGearSteerRad);

  // calculate distance from platform center to wheel center
  m_dExWheelDistMM = sqrt((m_dExWheelXPosMM * m_dExWheelXPosMM) + (m_dExWheelYPosMM * m_dExWheelYPosMM));

  // calculate direction of rotational vector
  m_dExWheelAngRad = MathSup::atan4quad(m_dExWheelYPosMM, m_dExWheelXPosMM);

  m_dVelWheelMMS = geom_.dRadiusWheelMM * (state_.dVelGearDriveRadS - dFactorVel * state_.dVelGearSteerRadS);
}

double WheelData::mergeRotRobRadS(const WheelData& wheel1, const WheelData& wheel2)
{
  // calc Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
  double dtempDiffXMM = wheel2.m_dExWheelXPosMM - wheel1.m_dExWheelXPosMM;
  double dtempDiffYMM = wheel2.m_dExWheelYPosMM - wheel1.m_dExWheelYPosMM;

  double dtempRelDistWheelsMM = sqrt(dtempDiffXMM * dtempDiffXMM + dtempDiffYMM * dtempDiffYMM);
  double dtempRelPhiWheelsRAD = MathSup::atan4quad(dtempDiffYMM, dtempDiffXMM);

  // transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
  double dtempRelPhiWheel1RAD = wheel1.state_.dAngGearSteerRad - dtempRelPhiWheelsRAD;
  double dtempRelPhiWheel2RAD = wheel2.state_.dAngGearSteerRad - dtempRelPhiWheelsRAD;

  return (wheel2.m_dVelWheelMMS * sin(dtempRelPhiWheel2RAD) - wheel1.m_dVelWheelMMS * sin(dtempRelPhiWheel1RAD)) /
         dtempRelDistWheelsMM;
}

double WheelData::getVelX() const
{
  return m_dVelWheelMMS * cos(state_.dAngGearSteerRad);
}
double WheelData::getVelY() const
{
  return m_dVelWheelMMS * sin(state_.dAngGearSteerRad);
}
}  // namespace cob_omni_drive_controller
