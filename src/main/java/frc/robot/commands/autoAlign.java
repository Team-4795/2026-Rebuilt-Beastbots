package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class autoAlign extends Command {
  public static Translation2d hubPosition;
  public Translation2d shuttlePos1;
  public Translation2d shuttlePos2;

  public static double goalAngle = 0;
  private final double shootVelocity = 3; // M/S

  public static Translation2d lookGoal;
  public static final PIDController controller = new PIDController(1.3, 0.5, 0.0);
  private Alliance currentAlliance;
  public static autoAlign instance;
  private Drive drive;
  private double controllerLeftX;
  private double controllerLeftY;

  @Override
  public void initialize() {
    currentAlliance = DriverStation.getAlliance().get();
    drive = Drive.getInstance();
    if (currentAlliance == Alliance.Red) {
      hubPosition = Constants.FieldConstants.redHub;
      shuttlePos2 = new Translation2d(Constants.FieldConstants.fieldLength - 2, 1.5);
      shuttlePos1 =
          new Translation2d(
              Constants.FieldConstants.fieldLength - 2, Constants.FieldConstants.fieldlWidth - 1.5);
    } else {
      hubPosition = Constants.FieldConstants.blueHub;
      shuttlePos2 = new Translation2d(2, 1.5);
      shuttlePos1 = new Translation2d(2, (Constants.FieldConstants.fieldlWidth - 1.5));
    }
  }

  public void getControllerInputs(double leftX, double leftY) {
    controllerLeftX = leftX;
    controllerLeftY = leftY;
  }

  public void autoAlignAngle(double angle) {
    DriveCommands.setRotationGoal(drive, () -> controllerLeftX, () -> controllerLeftY, () -> angle);
  }

  public void aimAtPosition(Translation2d pos) {
    lookGoal = pos;

    Logger.recordOutput("current Target goal:", new Pose2d(pos, new Rotation2d(0, 0)));

    double distance = drive.getPose().getTranslation().getDistance(pos);
    double timeToHub = distance / shootVelocity;

    ChassisSpeeds chassisSpeed =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    double offsetX = chassisSpeed.vxMetersPerSecond * timeToHub * 0.3048f;

    double offsetY = chassisSpeed.vyMetersPerSecond * timeToHub * 0.3048f;

    Translation2d offsetHub = pos.plus(new Translation2d(-offsetX, -offsetY));

    Logger.recordOutput("offset Target goal:", new Pose2d(offsetHub, new Rotation2d(0, 0)));

    goalAngle =
        Math.atan2(
                offsetHub.getX() - drive.getPose().getX(),
                -(offsetHub.getY() - drive.getPose().getY()))
            - Math.PI / 2;
  }

  @Override
  public void execute() {

    if (currentAlliance != Alliance.Red) {
      shuttlePos2 =
          new Translation2d(
              Constants.FieldConstants.fieldLength - 2, (2) * 1.35 - drive.getPose().getY() * 0.35);
      shuttlePos1 =
          new Translation2d(
              Constants.FieldConstants.fieldLength - 2,
              (Constants.FieldConstants.fieldlWidth - 2) * 1.35 - drive.getPose().getY() * 0.35);
    } else {
      shuttlePos2 = new Translation2d(2, (1.5) * 1.35 - drive.getPose().getY() * 0.35);
      shuttlePos1 =
          new Translation2d(
              2,
              (Constants.FieldConstants.fieldlWidth - 1.5) * 1.35 - drive.getPose().getY() * 0.35);
    }
    if (drive.getPose().getX() > Constants.FieldConstants.blueHub.getX()
        && drive.getPose().getX() < Constants.FieldConstants.redHub.getX()) {
      if (drive.getPose().getY() > Constants.FieldConstants.fieldlWidth / 2.0) {
        aimAtPosition(shuttlePos1);
      } else {
        aimAtPosition(shuttlePos2);
      }
    } else {
      aimAtPosition(hubPosition);
    }
  }
}
