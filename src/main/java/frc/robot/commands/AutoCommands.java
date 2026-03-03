package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.drive.Drive;

public class AutoCommands {
  private static Shooter shooter = Shooter.getInstance();
  private static Drive drive = Drive.getInstance();

  private AutoCommands() {}

  public static Command shootDynamic() {
    double distance = drive.getDistanceToRedHub();
    return Commands.run(() -> shooter.setGoalDynamic(distance), shooter);
  }
}
