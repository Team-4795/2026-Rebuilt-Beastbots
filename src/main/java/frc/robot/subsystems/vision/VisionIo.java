package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIo {
    @AutoLog
    public static class VisionIoInputs
    {
        
    }
    public default void updateInputs(VisionIoInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}
}
