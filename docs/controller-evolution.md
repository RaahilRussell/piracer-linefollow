# Controller evolution

## 1. PD: react to image error

The first controller measured where an orange boundary crossed a lookahead row. Proportional control turned toward the current error; derivative control reacted to the error's rate of change and reduced oscillation.

PD was simple and effective enough to drive real laps. Its weakness was that one image-row position does not identify the path ahead. A straight and a bend can produce the same local error while requiring different steering angles.

## 2. Pure pursuit: steer toward a physical goal

Pure pursuit projects visible boundary points onto the floor, shifts them to the car's ride line, and chooses a lookahead point. Steering is computed from the curvature needed to reach that point.

This improved curve entry and turn holding because the controller used the shape of the visible path rather than only the current pixel displacement. It also made camera-model and delay calibration important: a wrong height, pitch, wheelbase, or lookahead can create a confident but wrong geometric target.

The current system keeps a small PD correction on top of pursuit. That correction handles residual image error caused by imperfect calibration and model mismatch. If a usable pursuit goal cannot be formed, the controller can fall back to PD.

## 3. Racing Line: plan where the car should drive

Fixed-offset pursuit still asked the car to follow the same ride line through straights and bends. Racing Line makes that offset dynamic: wider on the straight, closer to the inside tape near the apex, and constrained by the car's minimum turning radius.

That is the conceptual change from line following to autonomous racing. The detector answers “where is the track?” The Racing Line answers “where should the car be on the track?” Pure pursuit then turns that planned path into a steering command.

## What is real and what is simulated

Physical laps are the authority. The project had less than a week, busy schedules, and barely any access to the car, so the simulator became more elaborate than the available physical validation time justified. It remains useful for regression checks, controller comparisons, and identifying obviously unsafe settings, but simulated lap time is never presented as proof of physical performance.
