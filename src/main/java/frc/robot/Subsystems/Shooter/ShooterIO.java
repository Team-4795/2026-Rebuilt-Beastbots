package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;
  }

  public default void setVoltage(double voltage) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
