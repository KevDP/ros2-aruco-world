# Visibility: which markers the robot reads from each spot

216 spots on a grid over the free floor, 12 headings each: 2592 poses. Largest gap between the pose asked for and the pose Gazebo reported: 0.0 cm and 0.05 deg.

## How much of the room sees markers

| Distinct markers read from the spot, turning in place | Spots | Share |
|---|---:|---:|
| 0 | 29 | 13.4% |
| 1 | 48 | 22.2% |
| 2 | 40 | 18.5% |
| 3 | 38 | 17.6% |
| 4 | 24 | 11.1% |
| 5 | 16 | 7.4% |
| 6 | 10 | 4.6% |
| 7 | 8 | 3.7% |
| 8 or more | 3 | 1.4% |

Poses (spot and heading) with at least one marker read: 583 of 2592 (22.5%). Median share of headings that read a marker, per spot: 17%.

## Each marker's reach

True distance and viewing angle (0 is head-on) at which it was read; the last column compares the detector's distance with the true one.

| Id | Placement | Read from (share of spots) | True distance, min / median / max (m) | Viewing angle, median / max (deg) | Detector distance error, median |
|---|---|---:|---|---|---:|
| 0 | facing the robot | 23% | 1.3 / 2.4 / 4.3 | 24 / 62 | +3.2% |
| 1 | turned 30 degrees | 36% | 1.4 / 3.2 / 6.1 | 25 / 64 | +3.0% |
| 2 | turned 60 degrees | 29% | 0.9 / 2.7 / 5.9 | 25 / 75 | +4.2% |
| 3 | on the floor | 7% | 0.7 / 0.9 / 1.4 | 74 / 80 | +1.7% |
| 4 | on the side of a box | 31% | 0.8 / 3.0 / 5.2 | 20 / 71 | +1.8% |
| 5 | far wall | 43% | 2.2 / 4.5 / 8.2 | 22 / 71 | +14.1% |
| 6 | far wall, high, tilted down | 19% | 4.5 / 5.4 / 7.6 | 17 / 42 | +12.6% |
| 7 | left wall | 27% | 2.7 / 4.0 / 7.3 | 21 / 61 | +6.3% |
| 8 | right wall, high, tilted down | 13% | 5.2 / 6.1 / 8.1 | 15 / 39 | +14.0% |
| 9 | back wall, behind the robot | 31% | 2.2 / 3.5 / 7.7 | 24 / 68 | +4.3% |
