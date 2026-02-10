package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionIoReal implements VisionIo {

  PhotonCamera camera;
  PhotonPoseEstimator estimator;
  PhotonTargetSortMode sortMode;
  List<PhotonPipelineResult> result;

  public VisionIoReal(int camIndex) {
    camera = new PhotonCamera(VisionConstants.cameraNames[camIndex]);

    estimator =
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout2, VisionConstants.positionOfCameras[camIndex]);

    sortMode = PhotonTargetSortMode.Largest;
  }

  @Override
  public void updateInputs(VisionIoInputs inputs) {

  }
}
