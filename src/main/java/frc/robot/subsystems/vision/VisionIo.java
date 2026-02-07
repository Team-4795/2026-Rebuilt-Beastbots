package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIo {
  @AutoLog
  public static class VisionIoInputs {
    Pose3d[] pose = new Pose3d[] {};
    double[] timestamp = new double[] {};
    int[] tags = new int[] {};
  }

  public default void updateInputs(VisionIoInputs inputs) {}

  public default void setReferencePose(Pose2d reference) {}
}
