#pragma once
#include <vector>
#include "cob_omni_drive_controller/undercarriage/controller/controller_base.h"
namespace cob_omni_drive_controller
{
class UndercarriageCtrl : public UndercarriageCtrlBase<PosCtrlData>
{
public:
  typedef WheelCtrlPosParams WheelParams;
  void configure(const std::vector<PosCtrlParams>& pos_ctrl);
  UndercarriageCtrl(const std::vector<WheelParams>& params) : UndercarriageCtrlBase(params)
  {
  }
};
}  // namespace cob_omni_drive_controller
