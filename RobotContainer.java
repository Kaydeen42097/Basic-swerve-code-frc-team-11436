package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();

  // F310: set the switch on the back to "X" (XInput) so it works like an Xbox controller
  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    drive.setDefaultCommand(drive.run(() -> {
      // Stick up is negative on the controller, so flip it. Left on the stick is negative X, so flip that too.
      double x   = -MathUtil.applyDeadband(driver.getLeftY(),  Constants.DEADBAND);
      double y   = -MathUtil.applyDeadband(driver.getLeftX(),  Constants.DEADBAND);
      double rot = -MathUtil.applyDeadband(driver.getRightX(), Constants.DEADBAND);

      // Hold left bumper for robot-oriented driving
      boolean fieldRelative = !driver.leftBumper().getAsBoolean();

      drive.drive(
          x * Constants.MAX_SPEED_MPS,
          y * Constants.MAX_SPEED_MPS,
          rot * Constants.MAX_ANGULAR_RAD_PER_SEC,
          fieldRelative);
    }));

    // Y button: reset which way the robot considers "forward" (point it away from you first)
    driver.y().onTrue(drive.runOnce(drive::zeroGyro));
  }

  /** No autonomous routine yet. Required by the template's Robot.java. */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
