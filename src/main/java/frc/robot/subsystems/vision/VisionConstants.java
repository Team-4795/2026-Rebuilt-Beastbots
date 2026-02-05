package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
    public static Transform3d positionOfCamera = new Transform3d(
        new Translation3d(5,5,5 ), // in meters
        new Rotation3d(5,5,5) // in radians
    );

    public static AprilTagFieldLayout aprilTagFieldLayout;
}
