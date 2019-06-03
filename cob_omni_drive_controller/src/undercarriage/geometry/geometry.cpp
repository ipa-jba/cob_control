/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cob_omni_drive_controller/undercarriage/geometry/geometry.h>

#include <angles/angles.h>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace cob_omni_drive_controller
{
UndercarriageGeom::UndercarriageGeom(const std::vector<WheelParams> &params)
{
  for (std::vector<WheelParams>::const_iterator it = params.begin(); it != params.end(); ++it)
  {
    wheels_.push_back(std::make_shared<WheelData>(it->geom));
  }
}

void UndercarriageGeom::calcDirect(PlatformState &state) const
{
  UndercarriageGeomBase::calcDirect(state, wheels_);
}

void UndercarriageGeom::updateWheelStates(const std::vector<WheelState> &states)
{
  UndercarriageGeomBase::updateWheelStates(wheels_, states);
}

}  // namespace cob_omni_drive_controller
