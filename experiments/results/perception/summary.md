# Perception: how often the markers are read, and how well

2701 markers in view over 2592 frames, unhidden and with their white border
inside the frame. Rates below count markers within 45 degrees of head-on
unless a column says otherwise.

## Read rate by cell width

| Pixels per cell | Markers | Median distance (m) | whole cells | cell centres |
|---|---:|---:|---:|---:|
| 1.5-2 px | 209 | 7.5 | 2% | 10% |
| 2-2.5 px | 450 | 6.5 | 18% | 51% |
| 2.5-3 px | 325 | 5.3 | 34% | 86% |
| 3-3.5 px | 217 | 4.5 | 39% | 92% |
| 3.5-4 px | 130 | 3.9 | 54% | 97% |
| 4-5 px | 155 | 3.3 | 80% | 99% |
| 5-7 px | 121 | 2.5 | 98% | 100% |
| 7+ px | 56 | 1.6 | 100% | 100% |

## Read rate by distance

| Distance | Markers | whole cells | cell centres |
|---|---:|---:|---:|
| 0-1 m | 1 | 100% | 100% |
| 1-2 m | 54 | 100% | 100% |
| 2-3 m | 137 | 98% | 100% |
| 3-4 m | 241 | 66% | 98% |
| 4-5 m | 304 | 37% | 91% |
| 5-6 m | 327 | 30% | 77% |
| 6-8 m | 562 | 15% | 41% |

## Read rate within 4 m, by viewing angle

| Viewing angle | Markers | whole cells | cell centres |
|---|---:|---:|---:|
| 0-45 deg | 433 | 81% | 99% |
| 45-60 deg | 143 | 42% | 93% |
| 60-75 deg | 141 | 19% | 45% |

## Error of the measured pose

Measured minus true, for every marker read: the median and the 10th to 90th percentile.

| Distance | Read | Distance error | Viewing angle error |
|---|---:|---:|---:|
| 1-2 m | 54 | +1.8% (+1.2 to +2.2) | +0.1 deg (-0.2 to +0.5) |
| 2-3 m | 137 | +2.6% (+2.0 to +4.4) | +0.3 deg (-1.3 to +2.9) |
| 3-4 m | 236 | +5.0% (+1.5 to +8.4) | +1.2 deg (-1.9 to +8.0) |
| 4-5 m | 276 | +9.1% (+3.0 to +16.4) | +3.6 deg (-1.9 to +12.7) |
| 5-6 m | 256 | +15.5% (+9.5 to +20.4) | +5.7 deg (-0.5 to +14.0) |
| 6-8 m | 245 | +19.3% (+13.3 to +26.8) | +8.7 deg (-1.0 to +19.7) |

## Detections with the wrong id, or where no marker is

- whole cells: 0 with the wrong id, 0 where no marker is
- cell centres: 0 with the wrong id, 0 where no marker is
