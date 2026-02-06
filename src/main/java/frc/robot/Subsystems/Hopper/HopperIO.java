package frc.robot.Subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public class HopperIOInputs {
    public double position = 0; // In rotations?
    public double voltage = 0;
    public double velocity = 0; // In RPM?
  }

  public default void setVoltage(double voltage) {}

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void updateMotionProfile() {}
}
