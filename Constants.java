package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  //CAN IDs (set these in Phoenix Tuner X / REV Hardware Client)
  // Order: Front Left, Front Right, Back Left, Back Right
  public static final int[] DRIVE_IDS   = {1, 4, 7, 10};  // Kraken X60 drive (Talon FX)
  public static final int[] STEER_IDS   = {2, 5, 8, 11};  // Kraken X60 steering (Talon FX)
  public static final int[] ENCODER_IDS = {3, 6, 9, 12};  // CANcoder

  // CANcoder offsets in radians (calibrate: see notes)
  public static final double[] ENCODER_OFFSETS_RAD = {-2.732, -2.769, -1.137, 0.276};

  public static final double TRACK_WIDTH_M = Units.inchesToMeters(27.5); // left-right built in function on math import converts our measurment in inches to m
  public static final double WHEEL_BASE_M  = TRACK_WIDTH_M; // forgot bot = square

  //  Wheel / gearing 
  public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0); // wheel diameter stock mk5n wheels
  public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
  public static final double DRIVE_GEAR_RATIO = 6.03; // mk5n gear ration 

  //  Inversions (flip if a module drives backwards or steers the wrong way)
  public static final boolean DRIVE_INVERTED = false; 
  public static final boolean STEER_INVERTED = true;

  // Limits (kept low for first tests) in meter per second
  public static final double MAX_SPEED_MPS = 1.0; // increase after testing
  public static final double MAX_ANGULAR_RAD_PER_SEC = Math.PI * 1.5;
  public static final double DRIVE_CURRENT_LIMIT_A = 60;
  public static final double STEER_CURRENT_LIMIT_A = 40;

  // Steering PID (runs on the roboRIO using the CANcoder angle) 
  public static final double STEER_KP = 0.4;
  public static final double STEER_KD = 0.0;
  public static final double DEADBAND = 0.15;
}
