# Copyright 2020 The dm_control Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or  implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
"""Walkers for Locomotion tasks."""

from dm_control.locomotion.walkers.ant import Ant
from dm_control.locomotion.walkers.cmu_humanoid import CMUHumanoidPositionControlled
from dm_control.locomotion.walkers.cmu_humanoid import CMUHumanoidPositionControlledV2020
from dm_control.locomotion.walkers.nao import NAOPositionControlled
# Import removed.
from dm_control.locomotion.walkers.jumping_ball import JumpingBallWithHead
from dm_control.locomotion.walkers.jumping_ball import RollingBallWithHead
from dm_control.locomotion.walkers.rodent import Rat
