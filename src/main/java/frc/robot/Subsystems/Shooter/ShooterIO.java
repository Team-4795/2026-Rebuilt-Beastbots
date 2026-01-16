package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double position1 = 0;
    public double voltage1 = 0;
    public double velocity1 = 0;

    public double position2 = 0;
    public double voltage2 = 0;
    public double velocity2 = 0;
  }

  public default void setVoltage(double voltage) {}

  public default void updateInputs(ShooterIOInputs inputs1) {}

  public default void updateMotionProfile() {}

  public default void setGoal(double RPM) {}
}
