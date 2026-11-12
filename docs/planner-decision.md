# Planner decision (R7.3)

Decision: OWN PLANNERS. OMPL not used.

## Proof attempt (one try, time-boxed)
- Python 3.13.15 (Windows 11), pip 26.2.1.
- `pip index versions ompl` : no versions found.
- `pip download ompl --no-deps` : "No matching distribution found for ompl".
- No wheel for this Python / platform. No conda, no source build (needs Boost + CMake, outside time box).
- No dependency added to requirements.txt.

## Consequence for R7.4
R7.4 builds RRT-Connect, RRT* and PRM in Python behind the planner interface
(same interface an OMPL backend would use later). Collision checks use
`app/planning/collision.py` (see `docs/collision-decision.md`).

## PoC evidence (own RRT-Connect, 2-DOF, circle obstacle)
`tests/python/test_planner_poc.py::test_rrt_connect_2dof_circle`
- Space [-2, 2]^2, circle radius 0.8 at origin, start (-1.5, 0), goal (1.5, 0), step 0.2, seed 0.
- Result: 16.9 ms, 23 waypoints, path length 4.241 (straight line 3.0).
- Path checked: endpoints exact, every waypoint and every edge (0.02 sampling) collision free.
- The OMPL variant of the test is `skipif` OMPL missing (skipped here).
