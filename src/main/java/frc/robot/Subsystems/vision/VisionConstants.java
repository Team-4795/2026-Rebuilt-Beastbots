package frc.robot.Subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;

public class VisionConstants {
  public static class robotInfo {
    Pose3d pos;
    double stddev;
  }

  public static Transform3d positionOfCameras[] =
      new Transform3d[] {
        new Transform3d(
            new Translation3d(0.060325, 0.244475, 0.581025), // in meters
            new Rotation3d(0, -0.261799, Math.PI) // in radians
            )
      };

  public static final String[] cameraNames =
      new String[] {
        "Cod_Wave",
        // "Camera ids are not that special",
        // "Kendrick LaBarge was another one from last year",
        // "and who came up with Cod Wave?"
      };

  public static final double fieldBorderMargin = 2;
  public static final double zMargin = 2;

  public static final double xyStdDevSingleTag = 0.08f;
  public static final double xyStdDevMultiTag = 0.018f;

  public static AprilTagFieldLayout aprilTagFieldLayout2;

  static {
    try {
      VisionConstants.aprilTagFieldLayout2 =
          AprilTagFieldLayout.loadFromResource(
              AprilTagFields.k2026RebuiltAndymark.m_resourceFile); // Placefolder
      VisionConstants.aprilTagFieldLayout2.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
