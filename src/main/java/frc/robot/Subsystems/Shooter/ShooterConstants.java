package frc.robot.Subsystems.Shooter;

public class ShooterConstants {
  public static final int motorPort1 = 0;
  public static final int motorPort2 = 0;
  public static final int maxAmps = 30;
  public static final int shooterVoltage = 5;
  // RPM
  public static final double maxVeloxity = 10;
  public static final double maxAcceleration = 3;

  public class PID {
    // Proportion
    public static int kP = 0;
    // Integral
    public static int kI = 0;
    // Derivative
    public static int kD = 0;

    // Overcome gravity
    public static int kG = 0;
    // Volts for a singular unit
    public static int kV = 0;
    // Overcome static friction
    public static int kS = 0;
  }
}
