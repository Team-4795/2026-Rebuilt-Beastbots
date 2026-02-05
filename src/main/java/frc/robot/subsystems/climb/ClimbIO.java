package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;

    public double goalVelocity = 0;
    public double setpointVelocity = 0;
  }

  public default void setVoltage(double voltage) {}
}
