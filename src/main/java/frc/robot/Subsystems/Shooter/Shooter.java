package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static Shooter instance;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public ShooterIO shooterIo;

  public static Shooter getInstance() {
    return instance;
  }

  public static Shooter Initialize(ShooterIO shooterIo) {
    if (instance == null) {
      instance = new Shooter(shooterIo);
    }
    return instance;
  }

  public void setGoal(double RPM) {
    shooterIo.setGoal(RPM);
  }

  public void setShooterVoltage(double volts) {
    shooterIo.setVoltage(volts);
  }

  public Shooter(ShooterIO io) {
    shooterIo = io;
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    Logger.processInputs("Shooter/Shooter", inputs);
  }
}
