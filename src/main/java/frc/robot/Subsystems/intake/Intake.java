package frc.robot.Subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double intakeSpeed = 0.0;
  public boolean isStoring = false;

  private static Intake instance;

  public static Intake getInstance() {
    return instance;
  }

  public static Intake initialize(IntakeIO io) {
    if (instance == null) {
      instance = new Intake(io);
    }
    return instance;
  }

  public Command intake() { // make intake thingy
    return Commands.none();
  }

  private Intake(IntakeIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public double getPosition() {
    return inputs.angularPositionRot;
  }

  public void setIntakeSpeed(double speed) {
    intakeSpeed = speed;
    io.setMotorSpeed(intakeSpeed);
  }

  public Command intakeSlow() {
    return startEnd(() -> setIntakeSpeed(IntakeConstants.slow), () -> setIntakeSpeed(0));
  }

  public Command reverse() {
    return startEnd(() -> setIntakeSpeed(IntakeConstants.reverse), () -> setIntakeSpeed(0));
  }

  public void outtake() {
    isStoring = false;
  }

  public double voltage() {
    return io.voltage();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Intake speed", intakeSpeed);
    Logger.recordOutput("Intake/hasGamepiece", voltage());
  }
}
