# Stopping: how much room the guard leaves

Guard: stop zone 0.3 m, slow zone 0.8 m at 0.15 m/s, side margin 0.05 m, 20 Hz. Base: top speed 0.6 m/s, braking at most 1.0 m/s^2. Lidar: 10 Hz.

Room is the shortest true distance between the robot's outline and anything solid. 5 mm or less counts as touching.

## Approaches, by speed

Room when the guard cut the speed (slow zone) and when it sent zero (stop zone), the speed just before the stop, and the distance covered after it.

| Speed | Trials | Stopped | Passed | Touched | Room at the cut | Room at the stop | Speed at the stop | After the stop, median (max) | Room at rest, median (min) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.15 m/s | 36 | 34 | 2 | 0 | 27.7 cm | 27.7 cm | 0.15 m/s | 1.1 cm (1.2) | 26.6 cm (8.2) |
| 0.30 m/s | 36 | 34 | 2 | 0 | 75.5 cm | 27.8 cm | 0.15 m/s | 1.1 cm (34.3) | 26.6 cm (6.2) |
| 0.45 m/s | 36 | 33 | 3 | 0 | 71.7 cm | 28.7 cm | 0.15 m/s | 1.1 cm (1.2) | 27.6 cm (11.5) |
| 0.60 m/s | 36 | 33 | 3 | 0 | 72.1 cm | 27.7 cm | 0.15 m/s | 1.1 cm (1.2) | 26.8 cm (11.6) |

## Approaches, by target

Walls by the angle between the heading and the wall's normal, the post by the offset of its centre from the robot's centre line, all speeds together.

| Target | Setting | Trials | Stopped | Passed | Room at rest, median (min) | Least room |
|---|---|---:|---:|---:|---:|---:|
| wall | 0 | 12 | 12 | 0 | 29.0 cm (28.3) | 28.3 cm |
| wall | 15 | 12 | 12 | 0 | 27.9 cm (26.7) | 26.7 cm |
| wall | 30 | 12 | 12 | 0 | 25.8 cm (24.8) | 24.8 cm |
| wall | 45 | 12 | 12 | 0 | 22.8 cm (21.9) | 21.9 cm |
| wall | 60 | 12 | 12 | 0 | 17.7 cm (17.2) | 17.2 cm |
| wall | 75 | 12 | 12 | 0 | 11.9 cm (11.5) | 11.5 cm |
| post | 0.0 | 12 | 12 | 0 | 28.2 cm (27.1) | 27.1 cm |
| post | 0.08 | 12 | 12 | 0 | 27.7 cm (26.1) | 26.1 cm |
| post | 0.16 | 12 | 12 | 0 | 27.6 cm (26.6) | 26.6 cm |
| post | 0.24 | 12 | 2 | 10 | 7.2 cm (6.2) | 6.2 cm |
| box | face | 12 | 12 | 0 | 28.9 cm (28.2) | 28.2 cm |
| box | corner | 12 | 12 | 0 | 23.9 cm (23.6) | 23.4 cm |

Moved again after coming to rest, with the command held: 1 of 144 approaches, at most 29.8 cm.

## A box appears in the way

Boxes that appeared inside the stop zone (0.3 m) and were not touched: the distance from the moment the box was in place to rest, and the time until the guard first cut the speed. The estimate comes from the rates alone: a full lidar period and a full guard cycle at full speed, then braking at the base's limit.

| Speed | Trials | Distance to rest, median (max) | Reaction, median (max) | Estimate |
|---|---:|---:|---:|---:|
| 0.15 m/s | 25 | 2.8 cm (3.8) | 0.109 s (0.174) | 3.4 cm |
| 0.30 m/s | 24 | 7.9 cm (9.9) | 0.112 s (0.177) | 9.0 cm |
| 0.45 m/s | 16 | 14.7 cm (17.6) | 0.100 s (0.165) | 16.9 cm |
| 0.60 m/s | 8 | 24.1 cm (28.0) | 0.099 s (0.167) | 27.0 cm |

Every appearance, by the room there was when the box appeared:

| Speed | Room when it appeared | Trials | Touched | Room at rest, median |
|---|---|---:|---:|---:|
| 0.15 m/s | 0-0.1 m | 5 | 0 | 6.9 cm |
| 0.15 m/s | 0.1-0.15 m | 5 | 0 | 12.4 cm |
| 0.15 m/s | 0.15-0.2 m | 5 | 0 | 17.4 cm |
| 0.15 m/s | 0.2-0.25 m | 5 | 0 | 22.0 cm |
| 0.15 m/s | 0.25-0.3 m | 5 | 0 | 27.2 cm |
| 0.15 m/s | 0.3-0.35 m | 5 | 0 | 28.6 cm |
| 0.15 m/s | 0.35-0.5 m | 5 | 0 | 29.1 cm |
| 0.15 m/s | 0.5-0.7 m | 5 | 0 | 29.1 cm |
| 0.30 m/s | 0-0.1 m | 5 | 1 | 1.2 cm |
| 0.30 m/s | 0.1-0.15 m | 5 | 0 | 6.1 cm |
| 0.30 m/s | 0.15-0.2 m | 5 | 0 | 12.1 cm |
| 0.30 m/s | 0.2-0.25 m | 5 | 0 | 16.8 cm |
| 0.30 m/s | 0.25-0.3 m | 5 | 0 | 22.8 cm |
| 0.30 m/s | 0.3-0.35 m | 5 | 0 | 26.4 cm |
| 0.30 m/s | 0.35-0.5 m | 5 | 0 | 28.5 cm |
| 0.30 m/s | 0.5-0.7 m | 5 | 0 | 28.3 cm |
| 0.45 m/s | 0-0.1 m | 5 | 5 | 0.0 cm |
| 0.45 m/s | 0.1-0.15 m | 5 | 4 | 0.0 cm |
| 0.45 m/s | 0.15-0.2 m | 5 | 0 | 5.5 cm |
| 0.45 m/s | 0.2-0.25 m | 5 | 0 | 10.4 cm |
| 0.45 m/s | 0.25-0.3 m | 5 | 0 | 15.0 cm |
| 0.45 m/s | 0.3-0.35 m | 5 | 0 | 17.4 cm |
| 0.45 m/s | 0.35-0.5 m | 5 | 0 | 28.6 cm |
| 0.45 m/s | 0.5-0.7 m | 5 | 0 | 28.7 cm |
| 0.60 m/s | 0-0.1 m | 5 | 5 | 0.0 cm |
| 0.60 m/s | 0.1-0.15 m | 5 | 5 | 0.0 cm |
| 0.60 m/s | 0.15-0.2 m | 5 | 5 | 0.0 cm |
| 0.60 m/s | 0.2-0.25 m | 5 | 2 | 1.3 cm |
| 0.60 m/s | 0.25-0.3 m | 5 | 0 | 4.9 cm |
| 0.60 m/s | 0.3-0.35 m | 5 | 0 | 8.5 cm |
| 0.60 m/s | 0.35-0.5 m | 5 | 0 | 21.4 cm |
| 0.60 m/s | 0.5-0.7 m | 5 | 0 | 29.1 cm |

## The box taken away

With the command still held, the box was taken away in 160 appearances. The guard let the robot go on in 160 of them, 1.06 s after (median) and 1.17 s at most. It waits for the way to stay clear 1 s, 0.05 m past the stop zone.
