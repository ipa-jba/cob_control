#pragma once

#include "cob_omni_drive_controller/primitives/wheel_data.h"

namespace cob_omni_drive_controller
{
struct CtrlData : public WheelData
{
  CtrlParams params_;

  double m_dAngGearSteerTargetRad;  // choosen alternativ for steering angle
  double m_dVelGearDriveTargetRadS;

  // calculate inverse kinematics
  void setTarget(const PlatformState& state);

  virtual void calcControlStep(WheelCommand& command, double dCmdRateS, bool reset);

  virtual void reset();
  void pickConfiguration(double steer_target, double drive_target);

  template <typename P>
  CtrlData(const P& params) : WheelData(params.geom), params_(params.ctrl)
  {
    state_.dAngGearSteerRad = params_.dWheelNeutralPos;
    updateState(WheelState());
    setTarget(PlatformState());
    m_dAngGearSteerTargetRad = params_.dWheelNeutralPos;
  }
};
}  // namespace cob_omni_drive_controller
