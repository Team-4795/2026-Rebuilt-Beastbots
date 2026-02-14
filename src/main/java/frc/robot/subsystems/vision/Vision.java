package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static Vision instance;
  private VisionIo currentIo[];
  private VisionIoInputsAutoLogged inputs[];
  private boolean[] shouldUpdate = new boolean[] {true};

  public static Vision getInstance() {
    return instance;
  }

  public static Vision createInstance(VisionIo... io) {
    if (instance == null) {
      instance = new Vision(io);
    }
    return instance;
  }

  public Vision(VisionIo visionIO[]) {
    currentIo = visionIO;
    inputs = new VisionIoInputsAutoLogged[visionIO.length];
    for (int i = 0; i < visionIO.length; i++) {
      inputs[i] = new VisionIoInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < currentIo.length; i++) {
      currentIo[i].updateInputs(inputs[i]);
      Logger.processInputs("VisionCamera " + i + ":", inputs[i]);
    }
    for (int i = 0; i < currentIo.length; i++) {
      for (int p = 0; p < inputs[i].pose.length; p++) {
        Pose3d robotPose = inputs[i].pose[p];

        if (robotPose.getX() < -VisionConstants.fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + VisionConstants.fieldBorderMargin
            || robotPose.getY() < -VisionConstants.fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldlWidth + VisionConstants.fieldBorderMargin
            || robotPose.getZ() < -VisionConstants.zMargin
            || robotPose.getZ() > VisionConstants.zMargin) {
          continue;
        }

        List<Pose3d> tagPoses = new ArrayList<>();
        for (int tag : inputs[i].tags) {
          VisionConstants.aprilTagFieldLayout2.getTagPose(tag).ifPresent(tagPoses::add);
        }

        if (tagPoses.isEmpty()) continue;

        double distance = 0.0;
        for (var tag : tagPoses) {
          distance += tag.getTranslation().getDistance(robotPose.getTranslation());
        }

        distance /= tagPoses.size();
        double xyStdDev =
            (tagPoses.size() == 1
                    ? VisionConstants.xyStdDevSingleTag
                    : VisionConstants.xyStdDevMultiTag)
                * Math.pow(distance, 2);
        Vector<N3> stddevs = VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(40));

        Logger.recordOutput("Vision/" + VisionConstants.cameraNames[i] + "/Avg distance", distance);
        Logger.recordOutput("Vision/" + VisionConstants.cameraNames[i] + "/xy std dev", xyStdDev);

        if (shouldUpdate[i]) {
          // Bro idk what to do here
          Drive.getInstance()
              .addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
          // info.pos = robotPose;
          // int number = 0;
          // for (int x = 0; x < stddevs.; x++)

          // info.stddev = stddevs;
        }
      }
    }
  }
}
