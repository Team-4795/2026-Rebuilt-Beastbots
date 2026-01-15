package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setGoal(double speed) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
