// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Vendored from autoware_ekf_localizer (src/string.hpp) without algorithmic changes.
// Only the namespace (autoware::ekf_localizer -> autoware::pose_twist_fusion::ekf_vendored),
// the include paths and the header guards were adjusted so that both packages can be
// loaded into the same process. See design_last.md §5.1.

#ifndef EKF_VENDORED__STRING_HPP_
#define EKF_VENDORED__STRING_HPP_

#include <string>

namespace autoware::pose_twist_fusion::ekf_vendored
{

inline std::string erase_leading_slash(const std::string & s)
{
  std::string a = s;
  if (a.front() == '/') {
    a.erase(0, 1);
  }
  return a;
}

}  // namespace autoware::pose_twist_fusion::ekf_vendored

#endif  // EKF_VENDORED__STRING_HPP_
