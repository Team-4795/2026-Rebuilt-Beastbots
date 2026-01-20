package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Intake instance;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public IntakeIO io;

  /**
   * Gets an instance of the {@link Intake} subsystem
   *
   * @return An instance of the {@link Intake} subsystem, or `null` if none exists
   */
  public static Intake getInstance() {
    return instance;
  }

  /**
   * Gets an instance of the {@link Intake} subsystem. If none exists, this creates one
   *
   * @param io
   * @return An instance of the {@link Intake} subsystem
   */
  public static Intake Initialize(IntakeIO io) {
    if (instance == null) {
      instance = new Intake(io);
    }
    return instance;
  }

  /**
   * Sets the voltage of the intake
   *
   * @param volts
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets velocity of the intake, in RPM using Feedforward and PID
   *
   * @param rpm Desired velocity in RPM
   */
  public void setVelocity(double rpm) {
    io.setGoal(rpm);
  }

  private Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateMotionProfile();
    io.updateInputs(inputs);
    Logger.processInputs("Intake info:", inputs);
  }
}
