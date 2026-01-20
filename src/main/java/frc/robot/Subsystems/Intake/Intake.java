package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Intake instance;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public IntakeIO io;

  public static Intake getInstance() {
    return instance;
  }

  public static Intake Initialize(IntakeIO io) {
    if (instance == null) {
      instance = new Intake(io);
    }
    return instance;
  }

  public void setIntakeVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Sets velocity, in RPM */
  public void setVelocity(double rpm) {
    io.setGoal(rpm);
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateMotionProfile();
    io.updateInputs(inputs);
    Logger.processInputs("Intake info:", inputs);
  }
}
