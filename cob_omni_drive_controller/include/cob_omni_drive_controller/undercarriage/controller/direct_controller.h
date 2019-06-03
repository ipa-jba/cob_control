#pragma once
#include <vector>
#include "cob_omni_drive_controller/undercarriage/controller/controller_base.h"
namespace cob_omni_drive_controller
{
class UndercarriageDirectCtrl : public UndercarriageCtrlBase<CtrlData>
{
public:
  typedef WheelCtrlParams WheelParams;
  UndercarriageDirectCtrl(const std::vector<WheelParams>& params) : UndercarriageCtrlBase(params)
  {
  }
};
}  // namespace cob_omni_drive_controller
