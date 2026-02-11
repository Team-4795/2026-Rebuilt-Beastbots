package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  public static Climb instance;
  public ClimbIO io;
  public ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private Climb(ClimbIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public static Climb Initialize(ClimbIO io) {
    if (instance == null) {
      instance = new Climb(io);
    }
    return instance;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public static Climb getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb info", inputs);
  }
}
