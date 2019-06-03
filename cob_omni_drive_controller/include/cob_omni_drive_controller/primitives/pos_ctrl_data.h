#pragma once

#include "cob_omni_drive_controller/primitives/ctrl_data.h"

namespace cob_omni_drive_controller
{
struct PosCtrlData : public CtrlData
{
  PosCtrlParams pos_params_;

  // previous Commanded deltaPhi e(k-1)
  // double m_dCtrlDeltaPhi; not used
  // previous Commanded Velocity u(k-1)
  double m_dCtrlVelCmdInt;

  virtual void calcControlStep(WheelCommand& command, double dCmdRateS, bool reset);

  virtual void reset();

  PosCtrlData(const WheelCtrlPosParams& params) : CtrlData(params), pos_params_(params.pos_ctrl), m_dCtrlVelCmdInt(0)
  {
    state_.dAngGearSteerRad = params_.dWheelNeutralPos;
    updateState(WheelState());
    setTarget(PlatformState());
    m_dAngGearSteerTargetRad = params_.dWheelNeutralPos;
  }
};
}  // namespace cob_omni_drive_controller
