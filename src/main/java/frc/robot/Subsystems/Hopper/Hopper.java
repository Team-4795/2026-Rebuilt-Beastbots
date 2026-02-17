package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public static Hopper instance;
  public HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  public HopperIO io;
  private boolean extended = false;

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

  /**
   * Toggles the extension state of the hopper
   *
   * @return Returns a command to toggle hopper state
   */
  public Command toggle() {
    return this.setExtended(!this.extended);
  }

  /**
   * Sets hoppper to be extended or not extended
   *
   * @param extended A boolean stating whether the hopper should be extended or not
   * @return Returns a command to set the extension state
   */
  public Command setExtended(boolean extended) {
    if (this.extended != extended) {
      if (extended) {
        return Commands.sequence(
            Commands.runOnce(() -> this.setVoltage(HopperConstants.EXTENSION_VOLTAGE), this),
            Commands.waitSeconds(HopperConstants.EXTENSION_WAIT),
            Commands.runOnce(() -> this.setVoltage(0), this));
      } else {
        return Commands.sequence(
            Commands.runOnce(() -> this.setVoltage(-HopperConstants.EXTENSION_VOLTAGE), this),
            Commands.waitSeconds(HopperConstants.EXTENSION_WAIT),
            Commands.runOnce(() -> this.setVoltage(0), this));
      }
    } else {
      return Commands.none();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper info:", inputs);
  }
}
