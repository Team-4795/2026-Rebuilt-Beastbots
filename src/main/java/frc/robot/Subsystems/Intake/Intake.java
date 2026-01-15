package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Intake instance;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public IntakeIO shooterIo;

  public static Intake getInstance() {
    return instance;
  }

  public static Intake Initialize(IntakeIO shooterIo) {
    if (instance == null) {
      instance = new Intake(shooterIo);
    }
    return instance;
  }

  public void setIntakeVoltage(double volts) {
    shooterIo.setVoltage(volts);
  }

  public Intake(IntakeIO io) {
    shooterIo = io;
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    Logger.processInputs("Intake info:", inputs);
  }
}
