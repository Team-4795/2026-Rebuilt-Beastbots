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

  public class PID {

    // Proportion
    public static double kP = 0.01;
    // Integral
    public static double kI = 0.0;
    // Derivative
    public static double kD = 0;

    // Overcome gravity
    public static double kG = 0.00;
    // Volts for a singular unit
    public static double kV = 0;
    // Overcome static friction
    public static double kS = 0.01;

    public static double kA = 0;
  }
}
