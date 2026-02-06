package frc.robot.Subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public static Hopper instance;
  public HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  public HopperIO io;

  /**
   * Gets an instance of the {@link Hopper} subsystem
   *
   * <p>src/main/java/frc/robot/Subsystems/Hopper @return An instance of the {@link Hopper}
   * subsystem, or `null` if none exists
   */
  public static Hopper getInstance() {
    return instance;
  }

  /**
   * Gets an instance of the {@link Hopper} subsystem. If none exists, this creates one
   *
   * @param io
   * @return An instance of the {@link Hopper} subsystem
   */
  public static Hopper Initialize(HopperIO io) {
    if (instance == null) {
      instance = new Hopper(io);
    }
    return instance;
  }

  /**
   * Sets the voltage of the hopper 
   *
   * @param volts
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  private Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper info:", inputs);
  }
}
