package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final TalonFX drive;
  private final TalonFX steer;
  private final CANcoder encoder;
  private final double offsetRad;

  private final PIDController steerPid =
      new PIDController(Constants.STEER_KP, 0, Constants.STEER_KD);
  private final VoltageOut driveRequest = new VoltageOut(0);
  private final VoltageOut steerRequest = new VoltageOut(0);

  public SwerveModule(int driveId, int steerId, int encoderId, double offsetRad) {
    this.offsetRad = offsetRad;

    drive = new TalonFX(driveId);
    TalonFXConfiguration driveCfg = new TalonFXConfiguration();
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveCfg.MotorOutput.Inverted = Constants.DRIVE_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveCfg.CurrentLimits.SupplyCurrentLimit = Constants.DRIVE_CURRENT_LIMIT_A;
    driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    drive.getConfigurator().apply(driveCfg);
    drive.setPosition(0);

    steer = new TalonFX(steerId);
    TalonFXConfiguration steerCfg = new TalonFXConfiguration();
    steerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerCfg.MotorOutput.Inverted = Constants.STEER_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    steerCfg.CurrentLimits.SupplyCurrentLimit = Constants.STEER_CURRENT_LIMIT_A;
    steerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    steer.getConfigurator().apply(steerCfg);

    encoder = new CANcoder(encoderId);

    steerPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Module angle in radians, 0 = wheel pointing forward, CCW positive. */
  public Rotation2d getAngle() {
    double rad = encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI - offsetRad;
    return new Rotation2d(MathUtil.angleModulus(rad));
  }

  /** Raw CANcoder reading in radians (use this to find offsets). */
  public double getRawAngleRad() {
    return encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
  }

  private double getDriveMeters() {
    return drive.getPosition().getValueAsDouble()
        / Constants.DRIVE_GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE_M;
  }

  private double getDriveMetersPerSec() {
    return drive.getVelocity().getValueAsDouble()
        / Constants.DRIVE_GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE_M;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSec(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desired) {
    Rotation2d current = getAngle();
    desired.optimize(current);      // take the shortest turn, reverse wheel if needed
    desired.cosineScale(current);   // slow down while the wheel is still turning

    double driveVolts = desired.speedMetersPerSecond / Constants.MAX_SPEED_MPS * 12.0;
    drive.setControl(driveRequest.withOutput(driveVolts));

    double out = steerPid.calculate(current.getRadians(), desired.angle.getRadians());
    steer.setControl(steerRequest.withOutput(MathUtil.clamp(out, -1.0, 1.0) * 12.0));
  }

  public void stop() {
    drive.setControl(driveRequest.withOutput(0));
    steer.setControl(steerRequest.withOutput(0));
  }
}
