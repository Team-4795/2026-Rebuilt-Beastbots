package frc.robot.Subsystems.Hopper;

public class HopperConstants {
  public static final int canIdL = 15;
  public static final int canIdR = 16;
  public static final int maxAmps = 30;

  // Feedforward + PID constants
  public static final double KI = 0.1;
  public static final double KP = 0.1;
  public static final double KD = 0.1;

  public static final double KG = 0.1;
  public static final double KS = 0.1;
  public static final double KV = 0.1;

  public static final double MAX_ACCELERATION = 0.5;
  public static final double MAX_VELOCITY = 0.5;
  public static final double EXTENSION_VOLTAGE = 0.5;
  public static final double EXTENSION_WAIT =
      0.67; // Time to wait for motor to extend the hopper during hopper extensions

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
    public static final double MAX_VELOCITY = 9000;
  }
}
