#pragma once

#include <string>

namespace cob_omni_drive_controller
{
struct PlatformState
{
  double dVelLongMMS;
  double dVelLatMMS;
  double dRotRobRadS;

  const double getVelX()
  {
    return dVelLongMMS / 1000.0;
  }
  const double getVelY()
  {
    return dVelLatMMS / 1000.0;
  }

  void setVelX(const double val)
  {
    dVelLongMMS = val * 1000.0;
  }
  void setVelY(const double val)
  {
    dVelLatMMS = val * 1000.0;
  }

  PlatformState() : dVelLongMMS(0), dVelLatMMS(0), dRotRobRadS(0)
  {
  }
};

struct WheelState
{
  double dVelGearDriveRadS;
  double dVelGearSteerRadS;
  double dAngGearSteerRad;
  WheelState() : dVelGearDriveRadS(0), dVelGearSteerRadS(0), dAngGearSteerRad(0)
  {
  }
};
struct WheelCommand : public WheelState
{
  double dAngGearSteerRadDelta;
  WheelCommand() : dAngGearSteerRadDelta(0)
  {
  }
};

struct JointLimits
{
  double lower;
  double upper;
  double velocity;
  double effort;
};

struct WheelGeom
{
  std::string steer_name, drive_name;

  /** Position of the Wheels' Steering Axis'
   *  in cartesian (X/Y) coordinates
   *  relative to robot coordinate System
   */
  double dWheelXPosMM;
  double dWheelYPosMM;

  double dSteerDriveCoupling;

  double dRadiusWheelMM;
  double dDistSteerAxisToDriveWheelMM;
  JointLimits limits;
};

struct CtrlParams
{
  double dWheelNeutralPos;

  double dMaxDriveRateRadpS;
  double dMaxSteerRateRadpS;
};

struct WheelCtrlParams
{
  WheelGeom geom;
  CtrlParams ctrl;
};

struct PosCtrlParams
{
  /** ------- Position Controller Steer Wheels -------
   * Impedance-Ctrlr Prms
   *  -> model Stiffness via Spring-Damper-Modell
   *  -> only oriented at impedance-ctrl (no forces commanded)
   *  m_dSpring   Spring-constant (elasticity)
   *  m_dDamp             Damping coefficient (also prop. for Velocity Feedforward)
   *  m_dVirtM    Virtual Mass of Spring-Damper System
   *  m_dDPhiMax  maximum angular velocity (cut-off)
   *  m_dDDPhiMax maximum angular acceleration (cut-off)
   */
  double dSpring, dDamp, dVirtM, dDPhiMax, dDDPhiMax;
};

struct WheelCtrlPosParams
{
  WheelGeom geom;
  CtrlParams ctrl;
  PosCtrlParams pos_ctrl;
};

}  // namespace cob_omni_drive_controller
