#include <assert.h>
#include <cob_omni_drive_controller/undercarriage/controller/position_controller.h>
namespace cob_omni_drive_controller
{
void UndercarriageCtrl::configure(const std::vector<PosCtrlParams>& pos_ctrls)
{
  assert(wheels_.size() == pos_ctrls.size());
  for (size_t i = 0; i < wheels_.size(); ++i)
  {
    wheels_[i]->pos_params_ = pos_ctrls[i];
  }
}
}  // namespace cob_omni_drive_controller
