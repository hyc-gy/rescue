# Position estimation and collision protection

Position integration is called by motor_task after get_encoder(), using only L0/R0.
Coordinates: metres; +X is initial forward, +Y is left; yaw is degrees CCW.
pose_set_origin(x,y,yaw) resets the local frame while stopped. It does not locate
bases or map image pixels into world coordinates. Odometry drifts with wheel slip.

Before using pose_manager.valid, calibrate Navigation/pose_estimator.h:
- Set left/right metres per count from measured travel / encoder counts.
- Forward travel must produce positive counts on both wheels; set SIGN accordingly.
- Measure track width and verify GYRO_SIGN with a CCW turn.
- Only then set POSE_CALIBRATED=1. Defaults are placeholders, not chassis measurements.
IMU frames have checksum/length validation and timestamps. After 200 ms without
fresh angles, heading uses differential wheel odometry. valid indicates calibrated
integration timing, not guaranteed absolute accuracy; check imu_fresh separately.

Safety/collision_guard.c is called every motor cycle. It detects each driven
wheel independently: absolute applied PWM >=120 and fewer than 2 counts for
500 ms latches a suspected stall. Tune thresholds for low speed and startup.
Stall is not proof of physical contact (encoder failure also triggers it), and
spinning wheels against an obstacle may not trigger it.

For actual contact fit bumper switches and configure verified spare GPIOs in
CubeMX. Override collision_bumper_read() in a board adapter, returning 1 if ANY
switch is pressed. The default weak function returns 0: no contact sensor fitted.
Use stable, nonblocking reads. Debounce is 20 ms. Do not choose pins without
checking board wiring, timer/UART assignments and electrical levels.

On fault, motor_set_pwm() forces zero for every motor channel, including direct
commands from grab_task. Faults stay latched until collision_guard_clear() is
explicitly called; reset is refused while a bumper is pressed. No autonomous
reverse is attempted without rear clearance sensing. Mission actions already in
a timed delay may resume after explicit reset; restart/replan the mission before
resuming competition. Physical contact duration is not a peer-robot classifier.
The rules' 10-second forced separation is performed by officials; this code does
not claim to implement relocation to the start area.

New first-level folders are collected by gc_rescue/CMakeLists.txt on reconfigure.
Reconfigure CMake, build with arm-none-eabi-gcc, then validate at low speed on the
actual chassis. Host tests covered travel, turn, origin, angle wrap, stale IMU,
single-wheel stall, latch and bumper debounce/reset. Full ARM build was not run:
the local machine has native GCC but lacks ARM GCC/newlib headers.
