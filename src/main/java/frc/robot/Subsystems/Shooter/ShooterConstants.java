package frc.robot.Subsystems.Shooter;

public class ShooterConstants {
  public static final int motorPort1 = 0;
  public static final int motorPort2 = 0;
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

  public static double distanceFunction(double x) {
    // RPM = distance^2+4

    double distanceRPM = x * x + 4;
    return distanceRPM;
  }

  public class PID {

    // Proportion if it ollicilates decrease it if it doesnt give it enough volts incresa
    public static double kP = 0.02;
    // Integral
    public static double kI = 0.0;
    // Derivative
    public static double kD = 0;

    // Overcome gravity
    public static double kG = 0.00;
    // Volts for a singular unit increase if undershoot
    public static double kV = 0.0016;

    // Overcome static friction
    public static double kS = 0.012;

    public static double kA = 0;
  }
}
