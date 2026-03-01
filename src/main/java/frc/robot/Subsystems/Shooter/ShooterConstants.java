package frc.robot.Subsystems.Shooter;

public class ShooterConstants {
  // combined shooter and intake motors
  public static final int motorPort1 = 10;
  public static final int motorPort2 = 12;

  public static final int motorPort3 = 14; // shooter only

  public static final int maxAmps = 30;
  public static final int shooterVoltage = 5;
  // RPM
  public static final double maxVeloxity = 10;
  public static final double maxAcceleration = 3;
  public static final double kMaxOutput = 0.5;

  public static final double allowedErr = 0.1;
  public static final double voltsPerRPM = 2000 / 39;
  // in Meters
  public static final double minDistance = 2;
  public static final double maxDistance = 8;

  public static final int currentLimit = 40;

  public static final double tolerableRpmRangeShooter = 100;

  public static double distanceFunction(double x) {
    // RPM = distance^2+4

    double distanceRPM = x * x + 4;
    return distanceRPM;
  }

  public class PID {

    // Proportion if it ollicilates decrease it if it doesnt give it enough volts incresa
    // Shooter 3 increase until goal if oscillates decrease
    public static double kP = 0.0;
    // Integral
    public static double kI = 0.0;
    // Derivative 4 change until oscillate
    public static double kD = 0.0;

    // Overcome gravity, it's not an arm or elevator so we probably don't need
    public static double kG = 0.0;
    // Volts for a singular unit increase if undershoot (2)
    public static double kV = 0.00213;

    // Overcome static friction 1 until motor moves a bit
    public static double kS = 0.12;
    //
    public static double kA = 0.0;

    // these arent tuned
    // Middle bar
    public static double kP2 = 0.0;
    public static double kI2 = 0.0;
    public static double kD2 = 0.0;

    public static double kV2 = 0.0;
    public static double kS2 = 1;
    public static double kA2 = 0.0;
  }
}
