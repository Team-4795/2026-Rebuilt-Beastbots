package frc.robot.Subsystems.Intake;

public class IntakeConstants {
  public static final int motorPort = 0;
  public static final int maxAmps = 30;
  public static final int intakeVoltage = 5;

  // Feedforward + PID constants
  public static final double KI = 0.1;
  public static final double KP = 0.1;
  public static final double KD = 0.1;

  public static final double KG = 0.1;
  public static final double KS = 0.1;
  public static final double KV = 0.1;

  public static final double MAX_ACCELERATION = 0.5;
  public static final double MAX_VELOCITY = 0.5;

  public static class SIM {
    // Simulation FF
    public static final double KG = 1.3;
    public static final double KV = 1.00;
    public static final double KA = 0.05;
    public static final double KS = 0.001;

    public static final double KP = 5;
    public static final double KI = 0;
    public static final double KD = 2;

    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_VELOCITY = 10;
  }
}
