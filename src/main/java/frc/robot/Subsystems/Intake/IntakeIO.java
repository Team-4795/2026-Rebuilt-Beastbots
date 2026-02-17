package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double position = 0; // In rotations?
    public double voltage = 0;
    public double velocity = 0; // In RPM?
  }

  public default void setVoltage(double voltage) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void updateMotionProfile() {}
}
