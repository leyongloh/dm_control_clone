# Copyright 2019 The dm_control Authors.
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

"""Multi-agent MuJoCo soccer environment."""

import enum

from dm_control import composer
from dm_control.locomotion import walkers
from dm_control.locomotion.soccer.boxhead import BoxHead
from dm_control.locomotion.soccer.humanoid import Humanoid
from dm_control.locomotion.soccer.nao import NAO
from dm_control.locomotion.soccer.initializers import Initializer
from dm_control.locomotion.soccer.initializers import UniformInitializer
from dm_control.locomotion.soccer.observables import CoreObservablesAdder
from dm_control.locomotion.soccer.observables import InterceptionObservablesAdder
from dm_control.locomotion.soccer.observables import MultiObservablesAdder
from dm_control.locomotion.soccer.observables import ObservablesAdder
from dm_control.locomotion.soccer.pitch import MINI_FOOTBALL_GOAL_SIZE
from dm_control.locomotion.soccer.pitch import MINI_FOOTBALL_MAX_AREA_PER_HUMANOID
from dm_control.locomotion.soccer.pitch import MINI_FOOTBALL_MIN_AREA_PER_HUMANOID
from dm_control.locomotion.soccer.pitch import Pitch
from dm_control.locomotion.soccer.pitch import RandomizedPitch
from dm_control.locomotion.soccer.soccer_ball import regulation_soccer_ball
from dm_control.locomotion.soccer.soccer_ball import SoccerBall
from dm_control.locomotion.soccer.task import MultiturnTask
from dm_control.locomotion.soccer.task import Task
from dm_control.locomotion.soccer.team import Player
from dm_control.locomotion.soccer.team import RGBA_BLUE
from dm_control.locomotion.soccer.team import RGBA_RED
from dm_control.locomotion.soccer.team import Team
from dm_control.locomotion.walkers.initializers import mocap, NoOpInitializer, nao_initializer
import numpy as np
import pdb


class WalkerType(enum.Enum):
  BOXHEAD = 0
  ANT = 1
  HUMANOID = 2
  NAO = 3


def _make_walker(name, walker_id, marker_rgba, walker_type=WalkerType.BOXHEAD):
  """Construct a BoxHead walker."""
  # pdb.set_trace()
  if walker_type == WalkerType.BOXHEAD:
    return BoxHead(
        name=name,
        walker_id=walker_id,
        marker_rgba=marker_rgba,
    )
  if walker_type == WalkerType.ANT:
    return walkers.Ant(name=name, marker_rgba=marker_rgba)
  if walker_type == WalkerType.HUMANOID:
    initializer = mocap.CMUMocapInitializer()
    return Humanoid(
        name=name,
        marker_rgba=marker_rgba,
        walker_id=walker_id,
        visual=Humanoid.Visual.JERSEY,
        initializer=initializer)
  if walker_type == WalkerType.NAO:
    # initializer = NoOpInitializer()
    initializer = nao_initializer.NAOInitializer()
    # initializer = mocap.CMUMocapInitializer()
    return NAO(
        name=name,
        marker_rgba=marker_rgba,
        walker_id=walker_id,
        visual=NAO.Visual.JERSEY,
        initializer=initializer)
  raise ValueError("Unrecognized walker type: %s" % walker_type)


def _make_players(team_size, walker_type):
  """Construct home and away teams each of `team_size` players."""
  home_players = []
  away_players = []
  for i in range(team_size):
    home_walker = _make_walker("home%d" % i, i, RGBA_BLUE, walker_type)
    home_players.append(Player(Team.HOME, home_walker))

    away_walker = _make_walker("away%d" % i, i, RGBA_RED, walker_type)
    away_players.append(Player(Team.AWAY, away_walker))
  return home_players
# return home_players + away_players


def _area_to_size(area, aspect_ratio=0.75):
  """Convert from area and aspect_ratio to (width, height)."""
  return np.sqrt([area / aspect_ratio, area * aspect_ratio]) / 2.


def load(team_size,
         time_limit=45.,
         random_state=None,
         disable_walker_contacts=False,
         enable_field_box=False,
         keep_aspect_ratio=False,
         terminate_on_goal=True,
         walker_type=WalkerType.BOXHEAD):
  """Construct `team_size`-vs-`team_size` soccer environment.

  Args:
    team_size: Integer, the number of players per team. Must be between 1 and
      11.
    time_limit: Float, the maximum duration of each episode in seconds.
    random_state: (optional) an int seed or `np.random.RandomState` instance.
    disable_walker_contacts: (optional) if `True`, disable physical contacts
      between walkers.
    enable_field_box: (optional) if `True`, enable physical bounding box for
      the soccer ball (but not the players).
    keep_aspect_ratio: (optional) if `True`, maintain constant pitch aspect
      ratio.
    terminate_on_goal: (optional) if `False`, continuous game play across
      scoring events.
    walker_type: the type of walker to instantiate in the environment.

  Returns:
    A `composer.Environment` instance.

  Raises:
    ValueError: If `team_size` is not between 1 and 11.
    ValueError: If `walker_type` is not recognized.
  """
  goal_size = None
  min_size = (32, 24)
  max_size = (48, 36)
  ball = SoccerBall(radius=0.03, mass=0.020, friction=(0.1,0.1,0.0))

  if walker_type == WalkerType.HUMANOID:
    goal_size = MINI_FOOTBALL_GOAL_SIZE
    num_walkers = team_size * 2
    min_size = _area_to_size(MINI_FOOTBALL_MIN_AREA_PER_HUMANOID * num_walkers)
    max_size = _area_to_size(MINI_FOOTBALL_MAX_AREA_PER_HUMANOID * num_walkers)
    ball = regulation_soccer_ball()

  if walker_type == WalkerType.NAO:
    goal_size = MINI_FOOTBALL_GOAL_SIZE
    num_walkers = team_size * 2
    min_size = _area_to_size(MINI_FOOTBALL_MIN_AREA_PER_HUMANOID * num_walkers)
    max_size = _area_to_size(MINI_FOOTBALL_MAX_AREA_PER_HUMANOID * num_walkers)
    ball = SoccerBall(radius=0.03, mass=0.020, friction=(0.1,0.1,0.0))

  task_factory = Task
  if not terminate_on_goal:
    task_factory = MultiturnTask

  return composer.Environment(
      task=task_factory(
          players=_make_players(team_size, walker_type),
          # players=[_make_walker(name='home0', walker_id=0 ,marker_rgba=[0.1, 0.1, 0.8, 1.0], walker_type=walker_type)],
          arena=Pitch(
              size=(2,3),
              field_box=enable_field_box,
              goal_size=(0.29, 0.68, 0.51)),
          ball=ball,
          disable_walker_contacts=disable_walker_contacts),
      time_limit=time_limit,
      random_state=random_state)
