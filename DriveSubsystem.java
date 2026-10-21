package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR

  // +X is forward, +Y is left
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d( Constants.WHEEL_BASE_M / 2,  Constants.TRACK_WIDTH_M / 2),  // FL
      new Translation2d( Constants.WHEEL_BASE_M / 2, -Constants.TRACK_WIDTH_M / 2),  // FR
      new Translation2d(-Constants.WHEEL_BASE_M / 2,  Constants.TRACK_WIDTH_M / 2),  // BL
      new Translation2d(-Constants.WHEEL_BASE_M / 2, -Constants.TRACK_WIDTH_M / 2)); // BR

  private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI); // change if plugged in by USB
  private final SwerveDriveOdometry odometry;
  private final Field2d field = new Field2d();

  public DriveSubsystem() {
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(
          Constants.DRIVE_IDS[i], Constants.STEER_IDS[i],
          Constants.ENCODER_IDS[i], Constants.ENCODER_OFFSETS_RAD[i]);
    }
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getModulePositions());
    SmartDashboard.putData("Field", field);
  }

  /** Drive with speeds in m/s and rad/s. xSpeed = forward, ySpeed = left, rot = CCW. */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  public void stop() {
    for (SwerveModule m : modules) m.stop();
  }

  public void zeroGyro() {
    gyro.reset();
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), new Pose2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] p = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) p[i] = modules[i].getPosition();
    return p;
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getModulePositions());
    field.setRobotPose(getPose());

    // Use these to find offsets: point all wheels straight forward, read raw values
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Swerve/Module " + i + " raw angle (rad)", modules[i].getRawAngleRad());
      SmartDashboard.putNumber("Swerve/Module " + i + " angle (deg)", modules[i].getAngle().getDegrees());
    }
    SmartDashboard.putNumber("Swerve/Gyro (deg)", gyro.getRotation2d().getDegrees());
  }
}
