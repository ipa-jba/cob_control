#pragma once
#include <memory>
#include "cob_omni_drive_controller/primitives/pos_ctrl_data.h"
#include "cob_omni_drive_controller/types.h"
#include "cob_omni_drive_controller/undercarriage/geometry/geometry_base.h"

namespace cob_omni_drive_controller
{
template <typename T>
class UndercarriageCtrlBase : public UndercarriageGeomBase
{
public:
  // Constructor
  template <typename T2>
  UndercarriageCtrlBase(const std::vector<T2>& params)
  {
    for (typename std::vector<T2>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
      wheels_.push_back(std::make_shared<T>(*it));
    }
  }

  // Get result of direct kinematics
  virtual void calcDirect(PlatformState& state) const
  {
    UndercarriageGeomBase::calcDirect(state, wheels_);
  }

  // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
  virtual void updateWheelStates(const std::vector<WheelState>& states)
  {
    UndercarriageGeomBase::updateWheelStates(wheels_, states);
  }
  // Set desired value for Plattform Velocity to UndercarriageCtrl (Sollwertvorgabe)
  void setTarget(const PlatformState& state)
  {
    for (size_t i = 0; i < wheels_.size(); ++i)
    {
      wheels_[i]->setTarget(state);
    }
  }

  // Get set point values for the Wheels (including controller) from UndercarriangeCtrl
  void calcControlStep(std::vector<WheelCommand>& commands, double dCmdRateS, bool reset)
  {
    commands.resize(wheels_.size());

    for (size_t i = 0; i < wheels_.size(); ++i)
    {
      wheels_[i]->calcControlStep(commands[i], dCmdRateS, reset);
    }
  }

  void reset()
  {
    for (size_t i = 0; i < wheels_.size(); ++i)
    {
      wheels_[i]->reset();
    }
  }

protected:
  std::vector<std::shared_ptr<T> > wheels_;
};
}  // namespace cob_omni_drive_controller
