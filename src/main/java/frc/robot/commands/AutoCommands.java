package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutoCommands {
  public static Shooter shooter = Shooter.getInstance();
  public static Intake intake = Intake.getInstance();

  public static Command startShooter() {
    try {
      return Commands.runOnce(() -> shooter.setShooterVoltage(0.5), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Start Shooter\" Failed"));
    }
  }

  public static Command stopShooter() {
    try {
      return Commands.runOnce(() -> shooter.setShooterVoltage(0), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Stop Shooter\" Failed"));
    }
  }

  public static Command startIntake() {
    try {
      return Commands.runOnce(() -> intake.setIntakeVoltage(0.5), intake);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Start Intake\" Failed"));
    }
  }

  public static Command stopIntake() {
    try {
      return Commands.runOnce(() -> intake.setIntakeVoltage(0), intake);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Stop Intake\" Failed"));
    }
  }

  public static Command startClimb() {
    return Commands.none();
  }
}
