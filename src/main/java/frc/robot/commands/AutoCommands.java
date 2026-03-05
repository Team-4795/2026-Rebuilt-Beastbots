package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
  private static Shooter shooter = Shooter.getInstance();
  public static Intake intake = Intake.getInstance();
  private static Drive drive = Drive.getInstance();

  public static Command startShooter() {
    try {
      return Commands.runOnce(() -> shooter.readyToShoot(4500), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Start Shooter\" Failed"));
    }
  }

  public static Command stopShooter() {
    try {
      return Commands.runOnce(() -> shooter.setGoalSimple(0), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Stop Shooter\" Failed"));
    }
  }

  public static Command startIntake() {
    try {
      return Commands.runOnce(() -> intake.setVoltage(0.5), intake);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Start Intake\" Failed"));
    }
  }

  public static Command stopIntake() {
    try {
      return Commands.runOnce(() -> intake.setVoltage(0), intake);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Stop Intake\" Failed"));
    }
  }

  public static Command startClimb() {
    return Commands.none();
  }

  public static Command shootDynamic() {
    try {
      double distance = drive.getDistanceToRedHub();
      Logger.recordOutput("dude", distance);
      return Commands.run(() -> shooter.setGoalDynamic(distance), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Shoot Dynamic\" Failed"));
    }
  }
}
