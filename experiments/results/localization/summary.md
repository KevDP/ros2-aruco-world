# Localization: where each marker's fix lands

2592 frames, 2713 markers in view. The live detector refines
corners 2 px to each side, and the localizer's reach is 4 m.

## Corner windows, within 4 m

Read: markers in view within the reach that the window reads. Distance error: median of
measured minus true distance, from 3 to 4 m. Turned: fixes whose marker orientation came
out more than 30 deg off. Far off: fixes more than 0.5 and 1 m from the truth. Covered: fixes inside the covariance's 95% ellipse.

| Window (px each side) | Read | Fixes | Distance error, 3-4 m | Rotation error, median | Turned | Position error, median | 90th percentile | > 0.5 m | > 1 m | Covered |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5 | 57.8% | 603 | +5.0% | 2.1 deg | 40 | 13.0 cm | 74 cm | 11.9% | 8.6% | 80.1% |
| 3 | 57.8% | 616 | +3.5% | 0.6 deg | 11 | 8.7 cm | 19 cm | 1.8% | 1.8% | 96.6% |
| 2 | 57.8% | 622 | +3.5% | 0.6 deg | 4 | 8.3 cm | 17 cm | 1.1% | 0.6% | 97.6% |
| 1 | 57.8% | 629 | +3.3% | 1.6 deg | 16 | 10.2 cm | 28 cm | 5.7% | 3.2% | 91.1% |

## The live window's fixes within 4 m

622 fixes, 97.6% inside the covariance's 95% ellipse, 0.6% more than 1 m from the truth.
Position error, median 8.3 cm and 90th percentile 17 cm. As a share of the true distance, median 3.0% and 90th percentile 5.2%.
Orientation more than 30 deg off: 4 of them, landing 4.7 to 6.4 m from the truth.

## By distance, every fix of the live window

Along: median error along the line of sight, as a share of the distance (negative is
farther from the marker than the truth). Across and heading: 90th percentile of the
absolute error. Covered: fixes inside the covariance's 95% ellipse, whose sigmas are
3% of the distance along the line of sight and the
distance times 0.015 rad per metre across it.

| Distance | Fixes | Position error, median | 90th percentile | Along | Across, p90 | Heading, p90 | Covered | > 1 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0-1 m | 16 | 1.0 cm | 1 cm | -0.6% | 1 cm | 0.7 deg | 100.0% | 0.0% |
| 1-2 m | 113 | 3.1 cm | 5 cm | -1.8% | 4 cm | 1.1 deg | 97.3% | 0.0% |
| 2-3 m | 236 | 7.3 cm | 12 cm | -2.6% | 8 cm | 1.6 deg | 98.3% | 0.4% |
| 3-4 m | 310 | 13.3 cm | 24 cm | -3.4% | 15 cm | 2.1 deg | 97.1% | 0.6% |
| 4-5 m | 320 | 25.9 cm | 57 cm | -4.5% | 55 cm | 6.7 deg | 86.6% | 5.3% |
| 5-6 m | 265 | 41.2 cm | 328 cm | -4.9% | 304 cm | 29.3 deg | 67.5% | 20.8% |
| 6-8 m | 241 | 81.4 cm | 489 cm | -5.0% | 466 cm | 37.7 deg | 36.9% | 44.0% |

## By viewing angle, the live window's fixes within 4 m

| Viewing angle | Fixes | Position error, median | 90th percentile | > 0.5 m | > 1 m | Covered |
|---|---:|---:|---:|---:|---:|---:|
| 0-15 deg | 88 | 12.6 cm | 27 cm | 2.3% | 0.0% | 96.6% |
| 15-30 deg | 187 | 8.2 cm | 15 cm | 0.5% | 0.0% | 99.5% |
| 30-45 deg | 126 | 8.2 cm | 14 cm | 0.0% | 0.0% | 100.0% |
| 45-60 deg | 135 | 8.9 cm | 19 cm | 2.2% | 2.2% | 96.3% |
| 60-90 deg | 86 | 5.0 cm | 17 cm | 1.2% | 1.2% | 93.0% |

## Spread per metre that covers 95% of the live window's fixes within 4 m

Two sigmas at the 95th percentile, each error divided by how it grows in the covariance.

| | Configured | Would cover 95% |
|---|---:|---:|
| range_sigma_per_m (along) | 0.03 | 0.024 |
| angle_sigma_per_m, from the error across | 0.015 | 0.012 |
| angle_sigma_per_m, from the heading | 0.015 | 0.011 |

## The frame in fixes-top.png: 1583

Along is positive toward the marker, as in the table by distance.

| Marker | True distance | Position error | Along | Across | Inside the 95% ellipse |
|---:|---:|---:|---:|---:|---|
| 4 | 1.06 m | 2.2 cm | -1.6 cm | +1.5 cm | yes |
| 5 | 3.75 m | 14.9 cm | -12.9 cm | -7.4 cm | yes |

Ids read that the map does not have: none.
