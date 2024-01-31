# Copyright 2021 The dm_control Authors.
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

"""Initializers for walkers that use motion capture data."""

from dm_control.locomotion.mocap import cmu_mocap_data
from dm_control.locomotion.mocap import loader
from dm_control.locomotion.walkers import initializers
import pdb


_NAO_JOINTS_POS = [0, 0, 0]
_NAO_JOINTS_VEC = [0, 0, 0]
# _NAO_JOINTS_POS = [-0.17184996604919434, 0.16716408729553223, 1.7502521276474, 0.3282339572906494, -1.435865879058838, -0.3681180477142334, 0.07359004020690918, 0.060000061988830566, -0.06745409965515137, -0.1809699535369873, -0.39573001861572266, 2.100004196166992, 0.15642595291137695, 0.6259140968322754, -0.06745409965515137, -0.08739614486694336, 0.2070479393005371, 1.058502197265625, -1.186300277709961, 0.05119016021490097, 1.5907998085021973, 0.08739614486694336, 1.2777800559997559, 0.9526557922363281, 0.01222991943359375, 0.057199954986572266]
# _NAO_JOINTS_VEC = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
_NAO_VEC = [0, 0, 0]
_NAO_ANG_VEC = [0, 0, 0]

class NAOInitializer(initializers.UprightInitializer):
  """Initializer that uses data from a CMU mocap dataset.

     Only suitable if walker matches the motion capture data.
  """

  def __init__(self, mocap_key='CMU_077_02', version='2019'):
    """Load the trajectory."""
    ref_path = cmu_mocap_data.get_path_for_cmu(version)
    self._loader = loader.HDF5TrajectoryLoader(ref_path)
    self._trajectory = self._loader.get_trajectory(mocap_key)

  def initialize_pose(self, physics, walker, random_state):
    super(NAOInitializer, self).initialize_pose(
        physics, walker, random_state)
    physics.bind(walker.mocap_joints).qpos = _NAO_JOINTS_POS
    physics.bind(walker.mocap_joints).qvel = _NAO_JOINTS_VEC
    walker.set_velocity(physics,
                        velocity=_NAO_VEC,
                        angular_velocity=_NAO_ANG_VEC)
