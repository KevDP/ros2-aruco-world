# Fusion: how far each estimate is from the truth

30 runs of the same route, steered by the true pose: 51.3 m in 228 s (median). Each run starts a fresh simulation with its own drift seed, and Gazebo draws a new gyro bias.

Sources: wheels is /wheel/odometry, local is /odometry/local (wheels and gyro), global is /odometry/global (the same plus the marker fixes). Values are the median over the runs, with the 90th percentile and the maximum in brackets.

## Along the route and at its end

| Source | Position error, mean over the route | Largest position error | Position error at the end | Heading error at the end |
|---|---:|---:|---:|---:|
| wheels | 36.1 cm (47, 63) | 85.2 cm (106, 147) | 42.1 cm (92, 102) | 8.4 deg (15.7, 20.2) |
| local | 11.0 cm (19, 30) | 23.5 cm (35, 58) | 20.3 cm (28, 51) | 0.5 deg (0.9, 1.0) |
| global | 12.3 cm (14, 16) | 31.4 cm (45, 47) | 8.9 cm (26, 29) | 0.2 deg (1.4, 3.3) |
| local_raw | 18.4 cm (43, 64) | 42.7 cm (98, 140) | 32.3 cm (90, 139) | 5.7 deg (18.2, 30.0) |
| global_raw | 12.5 cm (15, 16) | 33.4 cm (49, 58) | 10.4 cm (24, 34) | 0.5 deg (2.3, 3.4) |
| global_gate5 | 12.6 cm (14, 42) | 32.8 cm (48, 591) | 9.1 cm (27, 29) | 0.2 deg (1.6, 6.1) |
| global_x4 | 11.5 cm (13, 40) | 30.1 cm (37, 589) | 9.1 cm (20, 21) | 0.1 deg (1.2, 4.5) |
| global_x16 | 10.4 cm (12, 38) | 28.4 cm (30, 580) | 9.3 cm (15, 16) | 0.1 deg (0.7, 2.9) |
| global_2hz | 9.9 cm (14, 40) | 29.5 cm (94, 665) | 7.1 cm (11, 13) | 0.1 deg (0.2, 0.3) |

Position error as each lap ends, back at the first corner (median over the runs):

| Source | Lap 1 | Lap 2 | Lap 3 |
|---|---:|---:|---:|
| wheels | 31.5 cm | 34.6 cm | 41.4 cm |
| local | 10.9 cm | 13.7 cm | 20.4 cm |
| global | 11.4 cm | 11.3 cm | 8.6 cm |
| local_raw | 17.2 cm | 26.7 cm | 33.2 cm |
| global_raw | 13.3 cm | 11.6 cm | 10.2 cm |
| global_gate5 | 11.1 cm | 11.7 cm | 8.8 cm |
| global_x4 | 10.2 cm | 9.5 cm | 8.7 cm |
| global_x16 | 9.1 cm | 8.2 cm | 8.7 cm |
| global_2hz | 7.5 cm | 7.6 cm | 7.3 cm |

Before driving, the robot stands still at its start pose while the first fixes arrive. Error as the drive starts, median (largest):

| Source | Position error | Heading error |
|---|---:|---:|
| wheels | 0.0 cm (0.0) | 0.00 deg (0.00) |
| local | 0.0 cm (0.0) | 0.02 deg (0.06) |
| global | 9.0 cm (9.0) | 0.17 deg (0.19) |
| local_raw | 0.0 cm (0.0) | 0.07 deg (0.29) |
| global_raw | 9.0 cm (9.0) | 0.18 deg (0.58) |
| global_gate5 | 9.0 cm (9.0) | 0.19 deg (0.65) |
| global_x4 | 9.0 cm (9.0) | 0.18 deg (0.47) |
| global_x16 | 9.0 cm (9.0) | 0.18 deg (0.48) |
| global_2hz | 6.0 cm (6.3) | 0.66 deg (0.67) |

Markers in view (a fix stamped in the last 1 s): 22% of the route time, averaged over the runs.

## Gyro bias

The gyro's mean turn rate in the second and a half at rest before each drive.

Bias: median 1.6 deg/min, largest 7.9 deg/min. Over a median drive, the largest would turn the heading by 30.1 deg.

## Fixes

19542 fixes in all, 652 per run (median). Late by (arrival minus image stamp): median 29 ms, 95th percentile 36 ms, largest 46 ms.
Position error of the fixes: median 11.3 cm, 90th percentile 51 cm; 1917 (10%) more than 1 m off, in 30 runs.

By marker (measured distance and viewing angle are medians; the last column is the marker's share of every far-off fix):

| Marker | Fixes | Distance | Viewing angle | Position error, median | > 1 m | Of the far-off |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 392 | 1.9 m | 11 deg | 5.1 cm | 0.0% | 0% |
| 1 | 7293 | 2.9 m | 58 deg | 10.3 cm | 24.0% | 91% |
| 2 | 549 | 3.6 m | 59 deg | 24.8 cm | 0.4% | 0% |
| 3 | 35 | 1.7 m | 83 deg | 13.3 cm | 0.0% | 0% |
| 5 | 3121 | 3.7 m | 64 deg | 17.9 cm | 5.4% | 9% |
| 9 | 8152 | 3.0 m | 16 deg | 9.7 cm | 0.0% | 0% |
After each far-off fix, global changed its error by at most 17.8 cm (error after minus error before).
After each far-off fix, global_raw changed its error by at most 12.5 cm (error after minus error before).
After each far-off fix, global_gate5 changed its error by at most 311.2 cm (error after minus error before).
After each far-off fix, global_x4 changed its error by at most 555.9 cm (error after minus error before).
After each far-off fix, global_x16 changed its error by at most 562.4 cm (error after minus error before).
After each far-off fix, global_2hz changed its error by at most 554.3 cm (error after minus error before).

## Runs that went off

Runs where the estimate passed 1 m from the truth, the longest stretch it stayed past that, and its largest error in those runs.

| Source | Runs | Which | Longest stretch | Largest error |
|---|---:|---|---|---:|
| wheels | 8 | 2, 3, 13, 15, 24, 26, 28, 29 | 37.7 s (run 15, 167.8 to 205.5 s) | 1.48 m |
| local | 0 | | | |
| global | 0 | | | |
| local_raw | 3 | 7, 23, 30 | 25.0 s (run 23, 173.7 to 198.7 s) | 1.40 m |
| global_raw | 0 | | | |
| global_gate5 | 1 | 6 | 17.7 s (run 6, 179.7 to 197.4 s) | 5.99 m |
| global_x4 | 1 | 6 | 17.1 s (run 6, 179.9 to 197.0 s) | 5.89 m |
| global_x16 | 1 | 6 | 17.3 s (run 6, 179.9 to 197.2 s) | 5.82 m |
| global_2hz | 3 | 6, 10, 23 | 17.6 s (run 6, 179.5 to 197.1 s) | 6.65 m |
