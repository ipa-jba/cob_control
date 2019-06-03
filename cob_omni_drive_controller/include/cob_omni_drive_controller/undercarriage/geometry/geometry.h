#pragma once
#include <memory>
#include <vector>
#include "cob_omni_drive_controller/types.h"
#include "cob_omni_drive_controller/undercarriage/geometry/geometry_base.h"
namespace cob_omni_drive_controller
{
class UndercarriageGeom : public UndercarriageGeomBase
{
public:
  struct WheelParams
  {
    WheelGeom geom;
  };

  // Constructor
  UndercarriageGeom(const std::vector<WheelParams>& params);

  // Get result of direct kinematics
  virtual void calcDirect(PlatformState& state) const;

  // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
  virtual void updateWheelStates(const std::vector<WheelState>& states);

private:
  std::vector<std::shared_ptr<WheelData>> wheels_;
};
}  // namespace cob_omni_drive_controller
