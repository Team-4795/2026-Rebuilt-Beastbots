package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutoCommands {
  public static Shooter shooter = Shooter.getInstance();
  public static Intake intake = Intake.getInstance();

  public static Command startShooter() {
    return Commands.run(() -> shooter.setShooterVoltage(0.5), shooter);
  }

  public static Command stopShooter() {
    return Commands.run(() -> shooter.setShooterVoltage(0), shooter);
  }

  public static Command startIntake() {
    return Commands.run(() -> intake.setIntakeVoltage(0.5), intake);
  }

  public static Command stopIntake() {
    return Commands.run(() -> intake.setIntakeVoltage(0), intake);
  }

  public static Command startClimb() {
    return Commands.none();
  }
}
