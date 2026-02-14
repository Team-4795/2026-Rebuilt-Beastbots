package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class autoAlign extends Command {
  public static Translation2d currentGoalLookRotation;

  public static double goalAngle = 0;
  private final double shootVelocity = 1; // M/S

  private Alliance currentAlliance;
  public static autoAlign instance;
  private Drive drive;

  @Override
  public void initialize() {
    currentAlliance = DriverStation.getAlliance().get();
    drive = Drive.getInstance();
    if (currentAlliance == Alliance.Red) {
      currentGoalLookRotation = Constants.FieldConstants.redHub;
    } else {
      currentGoalLookRotation = Constants.FieldConstants.blueHub;
    }
  }

  @Override
  public void execute() {
    Logger.recordOutput("current Hub goal:", currentGoalLookRotation);

    double distance = drive.getPose().getTranslation().getDistance(currentGoalLookRotation);
    double timeToHub = distance / shootVelocity;

    ChassisSpeeds chassisSpeed = drive.getChassisSpeeds();

    double offsetX =
        chassisSpeed.vxMetersPerSecond
            * timeToHub
            * Math.cos(drive.getChassisSpeeds().omegaRadiansPerSecond);
    double offsetY =
        chassisSpeed.vyMetersPerSecond
            * timeToHub
            * Math.sin(drive.getChassisSpeeds().omegaRadiansPerSecond);

    Translation2d offsetHub = currentGoalLookRotation.plus(new Translation2d(offsetX, offsetY));

    Logger.recordOutput("offset Hub goal:", offsetHub);

    goalAngle =
        Math.atan2(
                currentGoalLookRotation.getX() - drive.getPose().getX(),
                -(currentGoalLookRotation.getY() - drive.getPose().getY()))
            - Math.PI / 2;
  }
}
