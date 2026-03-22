package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.drive.Drive;

public class AutoCommands {
  private static Shooter shooter = Shooter.getInstance();
  public static Intake intake = Intake.getInstance();
  private static Drive drive = Drive.getInstance();

  public static Command indexerFlippedTrue() {
    return Commands.run(
        () -> {
          Shooter.shouldIndexerBeFlipped = true;
        },
        shooter);
  }

  public static Command indexerFlippedFalse() {
    return Commands.run(
        () -> {
          Shooter.shouldIndexerBeFlipped = false;
        },
        shooter);
  }

  public static Command startShooter() {
    try {
      return Commands.run(() -> shooter.readyToShoot(4000), shooter).withTimeout(5);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Start Shooter\" Failed"));
    }
  }

  public static Command stopShooter() {
    try {
      return Commands.runOnce(() -> shooter.stopAll(), shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Stop Shooter\" Failed"));
    }
  }

  public static Command startIntake() {
    try {
      return Commands.runOnce(() -> shooter.intake(), shooter);
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
      return Commands.run(
              () -> {
                double distance = 0;
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                  distance = drive.getDistanceToBlueHub(); // CHANGE SOMETIME
                } else {
                  distance = drive.getDistanceToRedHub(); // CHANGE SOMETIME
                }
                shooter.setGoalDynamic(distance);
              },
              shooter)
          .withTimeout(5);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Shoot Dynamic\" Failed"));
    }
  }

  public static Command shootDynamicForever() {
    try {
      return Commands.run(
          () -> {
            double distance = 0;
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
              distance = drive.getDistanceToBlueHub(); // CHANGE SOMETIME
            } else {
              distance = drive.getDistanceToRedHub(); // CHANGE SOMETIME
            }
            shooter.setGoalDynamic(distance);
          },
          shooter);
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Command \"Shoot Dynamic\" Failed"));
    }
  }
}
