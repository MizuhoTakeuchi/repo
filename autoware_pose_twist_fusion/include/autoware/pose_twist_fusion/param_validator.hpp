// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__POSE_TWIST_FUSION__PARAM_VALIDATOR_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__PARAM_VALIDATOR_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"

#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// Result of design §6.2. `errors` is fatal (the node must fail to start rather
/// than run in a silently broken configuration); `warnings` / `infos` are only
/// reported through diagnostics.
struct ValidationResult
{
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  std::vector<std::string> infos;

  [[nodiscard]] bool ok() const { return errors.empty(); }
  [[nodiscard]] std::string error_message() const;
};

/// Validates every row of design §6.2. Pure function of the parameter struct so
/// that `test_param_validator` can cover all rows without a ROS context.
ValidationResult validate_parameters(const Parameters & params);

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__PARAM_VALIDATOR_HPP_
