package frc.robot.Subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;
    public double current = 0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
