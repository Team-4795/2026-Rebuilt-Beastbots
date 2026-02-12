package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIoReal implements VisionIo {

  PhotonCamera camera;
  PhotonPoseEstimator estimator;
  PhotonTargetSortMode sortMode;
  List<PhotonPipelineResult> result;

  public VisionIoReal(int camIndex) {
    camera = new PhotonCamera(VisionConstants.cameraNames[camIndex]);

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout2, VisionConstants.positionOfCameras[camIndex]);

    sortMode = PhotonTargetSortMode.Largest;
  }

  @Override
  public void updateInputs(VisionIoInputs inputs) {
    inputs.pipelineIndex = camera.getPipelineIndex();
    inputs.sortMode = sortMode.toString();

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      visionEst = estimator.estimateCoprocMultiTagPose(result); // Multitag estimation
      if (visionEst.isEmpty()) {
        visionEst =
            estimator.estimateLowestAmbiguityPose(result); // Singletag if only one tag is in view
      }

      // Update logs if we're getting vision data
      visionEst.ifPresentOrElse(
          (pose) -> {
            inputs.pose = new Pose3d[] {pose.estimatedPose};
            inputs.timestamp = new double[] {pose.timestampSeconds};
            inputs.tags =
                pose.targetsUsed.stream()
                    .mapToInt(
                        (target) -> {
                          return target.getFiducialId();
                        })
                    .toArray();
          },
          () -> {
            inputs.pose = new Pose3d[] {};
            inputs.timestamp = new double[] {};
            inputs.tags = new int[] {};
          });
    }
  }
}
