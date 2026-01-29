package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutoCommands {
  private static final Shooter shooter = Shooter.getInstance();

  public static Command setGoalOuttake(double RPM) {
    return Commands.run(() -> shooter.setGoal(RPM), shooter);
  }
  public static Command climbAuto()
  {
        return Commands.none();
  }
}
