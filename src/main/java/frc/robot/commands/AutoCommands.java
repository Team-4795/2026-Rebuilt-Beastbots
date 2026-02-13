package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutoCommands {
  public static Shooter shooter = Shooter.getInstance();

  public static Command startShooter() {
    return Commands.run(() -> shooter.setShooterVoltage(0.5), shooter);
  }

  public static Command stopShooter() {
    shooter.setShooterVoltage(0);
    return Commands.none();
  }

  public static Command startIntake() {
    return Commands.none();
  }

  public static Command climb() {
    return Commands.none();
  }
}
