FRC team 11436 Swerve Drive (Java / WPILib 2026)

A simple drivetrain for an FRC robot, written in Java with WPILib's command-based framework. It is meant as a clean starting point that is easy to read and tune.

Status: early code, written before the robot was finished. Not yet tested on hardware. Expect to tune and fix things.

Hardware
Part	Used
Swerve modules	SDS MK5n (6.03:1 drive ratio)
Drive motors	Kraken X60 (Talon FX)
Steer motors	Kraken X44 (Talon FX)
Steering encoders	CTRE CANcoder
Gyro	NavX2 (MXP port)
Controller	Logitech F310 (XInput mode)
Robot controller	roboRIO 2.0
Software requirements
WPILib 2026
Vendor libraries:
Phoenix 6 (Kraken and CANcoder)
Studica (NavX2)
Project structure
File	Purpose
Constants.java	CAN IDs, module geometry, gear ratio, limits, PID gains
SwerveModule.java	One module: drive motor, steer motor, CANcoder, steering PID
DriveSubsystem.java	Kinematics, gyro, odometry, and the drive() method
RobotContainer.java	Controller bindings and the default drive command

Copy these into src/main/java/frc/robot/ of a WPILib command-based project.

Setup
Assign CAN IDs in Phoenix Tuner X to match Constants.java (default order: front-left, front-right, back-left, back-right).
Module	Drive	Steer	CANcoder
Front left	1	2	3
Front right	4	5	6
Back left	7	8	9
Back right	10	11	12
Measure your robot and set TRACK_WIDTH_M (left-right) and WHEEL_BASE_M (front-back), center to center of the modules.
Set the F310 back switch to X (XInput).
Calibrate the CANcoder offsets (below).
Calibrating CANcoder offsets
Point all four wheels straight forward, with bevel gears on the same side on every module.
Deploy the code and open SmartDashboard or Shuffleboard.
Read Swerve/Module N raw angle (rad) for modules 0 to 3 (FL, FR, BL, BR).
Put those values in ENCODER_OFFSETS_RAD in Constants.java and redeploy.
All four Swerve/Module N angle (deg) values should now read close to 0.

Leave the CANcoder's own magnet offset at 0 in Tuner X, since the code applies the offset itself.

First test (robot on blocks)
If a wheel drives backwards, flip DRIVE_INVERTED.
If a module steers the wrong way or twitches, flip STEER_INVERTED.
Confirm all wheels point and spin the same way when pushing the stick forward.
Controls
Input	Action
Left stick	Drive (field-oriented)
Right stick X	Rotate
Left bumper (hold)	Robot-oriented driving
Y	Zero the gyro
Tuning
STEER_KP: lower if wheels wobble when turning; raise if they are sluggish.
MAX_SPEED_MPS: set to the robot's real top speed. Starts conservative at 3.0.
Drive is open-loop voltage control for now.
Roadmap
 Closed-loop drive velocity control
 Steering PID on the Krakens with a fused CANcoder
 Simulation support
 PathPlanner autonomous
 Limelight AprilTag pose estimation
