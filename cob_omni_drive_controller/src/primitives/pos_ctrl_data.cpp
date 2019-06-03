#include <cob_omni_drive_controller/primitives/pos_ctrl_data.h>
#include <cob_omni_drive_controller/utils/math_sup.h>
namespace cob_omni_drive_controller
{
void PosCtrlData::reset()
{
  m_dCtrlVelCmdInt = 0.0;
}

void PosCtrlData::calcControlStep(WheelCommand& command, double dCmdRateS, bool reset)
{
  CtrlData::calcControlStep(command, dCmdRateS, reset);
  if (!reset)
  {
    // Impedance-Ctrl
    // Calculate resulting desired forces, velocities
    // double dForceDamp, dForceProp, dAccCmd, dVelCmdInt;
    double dForceDamp = -pos_params_.dDamp * m_dCtrlVelCmdInt;
    double dForceProp = pos_params_.dSpring * command.dAngGearSteerRadDelta;

    double dAccCmd = (dForceDamp + dForceProp) / pos_params_.dVirtM;
    dAccCmd = MathSup::limitValue(dAccCmd, pos_params_.dDDPhiMax);

    double dVelCmdInt = m_dCtrlVelCmdInt + dCmdRateS * dAccCmd;
    dVelCmdInt = MathSup::limitValue(dVelCmdInt, pos_params_.dDPhiMax);

    // Store internal ctrlr-states
    m_dCtrlVelCmdInt = dVelCmdInt;

    // set outputs
    command.dVelGearSteerRadS = MathSup::limitValue(dVelCmdInt, params_.dMaxSteerRateRadpS);
  }
}
}  // namespace cob_omni_drive_controller
