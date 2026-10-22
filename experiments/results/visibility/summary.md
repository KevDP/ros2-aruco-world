# Visibility: which markers the robot reads from each spot

216 spots on a grid over the free floor, 12 headings each: 2592 poses. Largest gap between the pose asked for and the pose Gazebo reported: 0.0 cm and 0.04 deg.

## How much of the room sees markers

| Distinct markers read from the spot, turning in place | Spots | Share |
|---|---:|---:|
| 0 | 7 | 3.2% |
| 1 | 12 | 5.6% |
| 2 | 26 | 12.0% |
| 3 | 39 | 18.1% |
| 4 | 36 | 16.7% |
| 5 | 31 | 14.4% |
| 6 | 29 | 13.4% |
| 7 | 17 | 7.9% |
| 8 | 10 | 4.6% |
| 9 or more | 9 | 4.2% |

Poses (spot and heading) with at least one marker read: 971 of 2592 (37.5%). Median share of headings that read a marker, per spot: 33%.

## Each marker's reach

True distance and viewing angle (0 is head-on) at which it was read; the last column compares the detector's distance with the true one.

| Id | Placement | Read from (share of spots) | True distance, min / median / max (m) | Viewing angle, median / max (deg) | Detector distance error, median |
|---|---|---:|---|---|---:|
| 0 | facing the robot | 42% | 1.3 / 3.2 / 4.9 | 35 / 73 | +5.4% |
| 1 | turned 30 degrees | 52% | 1.4 / 3.7 / 6.4 | 27 / 75 | +4.1% |
| 2 | turned 60 degrees | 43% | 0.9 / 3.4 / 6.2 | 29 / 77 | +7.8% |
| 3 | on the floor | 8% | 0.7 / 0.9 / 1.6 | 74 / 81 | +1.6% |
| 4 | on the side of a box | 44% | 0.8 / 3.6 / 6.1 | 24 / 78 | +2.4% |
| 5 | far wall | 62% | 2.2 / 4.8 / 8.3 | 23 / 71 | +15.4% |
| 6 | far wall, high, tilted down | 39% | 4.5 / 5.7 / 8.3 | 15 / 50 | +14.0% |
| 7 | left wall | 41% | 2.7 / 4.5 / 7.5 | 23 / 63 | +8.9% |
| 8 | right wall, high, tilted down | 34% | 5.2 / 6.2 / 8.1 | 16 / 46 | +16.0% |
| 9 | back wall, behind the robot | 68% | 2.2 / 4.5 / 7.5 | 26 / 68 | +11.6% |
