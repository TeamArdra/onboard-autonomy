"""Reactive, memory-less "which direction has open space right now" --
pure logic, no rclpy.

This is the first real use of the RPLidar A2M12 anywhere in this repo --
today only `telemetry_bridge_node.py` touches `/scan`, and only to check
a message recently arrived, never to look at its ranges (see
CHECKPOINT-adjacent research notes / this module's caller,
`test_mission_policy.py`). It is deliberately NOT SLAM: no map, no
memory between calls, no localization -- it answers exactly one question,
"given this one scan, which direction can the vehicle safely step into
right now", the same kind of pure function `sensor_model.reveal_cells()`
is for the simulation (that one asks "what's visible from this pose in
the simulated ground truth"; this one asks the same shape of question
against a real, single, noisy scan).

Takes a plain `(ranges, angle_min, angle_increment)` tuple, never a raw
`sensor_msgs/LaserScan` -- the ROS boundary conversion belongs to the
calling node (`test_mission_node.py`), matching `ros_conversions.py`'s
existing OccupancyGrid-message-to-dict pattern, so this module stays
importable and unit-testable without a ROS environment, same as every
other pure-logic module in this package.

Conservative by construction, matching this repo's "raise/refuse rather
than fabricate" convention (see `flight_command_interface.py`'s
`position`/`velocity` docstrings): a ray is only ever treated as "open"
if its reported range is a genuine, in-range return that clears the
requested step distance plus a safety margin. A missing, non-finite, or
out-of-range reading is treated as UNKNOWN, never as "probably clear" --
unknown space is not open space. If nothing in the whole scan qualifies,
or the scan itself is empty, this returns `None`, and the caller's only
correct response is to hold in place, not guess a direction.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .geometry import wrap


@dataclass(frozen=True)
class GapResult:
    """One cycle's chosen "move this way" decision."""

    direction_rad: float  # sensor-frame bearing to move toward
    step_m: float  # how far to move this cycle (<= max_step_m)
    arc_width_rad: float  # width of the open arc this was chosen from


def find_open_direction(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    min_clearance_m: float,
    max_step_m: float,
    max_valid_range_m: float,
) -> Optional[GapResult]:
    """Pick the widest contiguous arc of rays whose range clears
    `max_step_m + min_clearance_m` (i.e. there is still `min_clearance_m`
    of buffer left even after moving the full `max_step_m` toward it), and
    return a direction toward its middle plus a capped step distance.

    Ties (equal widest-arc width) resolve to the arc with the greater
    average range, then to the lowest starting index, so this is fully
    deterministic for a given scan -- required for reproducible tests and
    for `abort()`/geofence review to reason about what the vehicle will do.

    A ray is only counted as usable if its reading is finite, positive,
    and at most `max_valid_range_m` -- see module docstring. `ranges` is
    treated as a full 360-degree sweep (wraparound between the last and
    first ray is honored when forming arcs), matching the RPLidar A2M12's
    own sweep; a narrower/partial scan degrades gracefully (wraparound
    grouping simply never triggers if the endpoints aren't both usable).
    """
    n = len(ranges)
    if n == 0:
        return None

    threshold = max_step_m + min_clearance_m
    usable = [
        math.isfinite(r) and 0.0 < r <= max_valid_range_m and r >= threshold
        for r in ranges
    ]
    if not any(usable):
        return None

    arcs = _contiguous_arcs(usable)

    def arc_avg_range(start: int, length: int) -> float:
        total = 0.0
        for k in range(length):
            total += ranges[(start + k) % n]
        return total / length

    best_start, best_len = max(
        arcs, key=lambda arc: (arc[1], arc_avg_range(arc[0], arc[1]), -arc[0])
    )

    mid_index = (best_start + (best_len - 1) / 2.0) % n
    direction = angle_min + mid_index * angle_increment

    # Every ray in the arc individually satisfies
    # r >= max_step_m + min_clearance_m (that's the usability test above),
    # so moving the full max_step_m always leaves at least min_clearance_m
    # of buffer -- the step distance is always exactly max_step_m for any
    # arc this function returns.
    return GapResult(
        direction_rad=wrap(direction),
        step_m=max_step_m,
        arc_width_rad=best_len * angle_increment,
    )


def _contiguous_arcs(usable: Sequence[bool]) -> list[tuple[int, int]]:
    """Return `[(start_index, length), ...]` for every maximal run of
    `True` in `usable`, treating it as circular (wraparound between the
    last and first element). If every element is `True`, returns a single
    arc covering the whole circle."""
    n = len(usable)
    if all(usable):
        return [(0, n)]

    arcs: list[tuple[int, int]] = []
    i = 0
    while i < n:
        if not usable[i]:
            i += 1
            continue
        start = i
        length = 0
        while length < n and usable[(start + length) % n]:
            length += 1
        arcs.append((start, length))
        i = start + length if start + length > i else i + 1
        if length == n:
            break

    # Merge a wraparound split: a run ending at n-1 and one starting at 0
    # are really one arc if both exist and are adjacent.
    if len(arcs) >= 2 and arcs[0][0] == 0 and arcs[-1][0] + arcs[-1][1] == n:
        first_start, first_len = arcs[0]
        last_start, last_len = arcs[-1]
        if first_len < n and last_len < n:
            merged_len = first_len + last_len
            if merged_len <= n:
                arcs[0] = (last_start, merged_len)
                arcs.pop()

    return arcs
