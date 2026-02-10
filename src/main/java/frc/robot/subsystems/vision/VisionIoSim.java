package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIoSim implements VisionIo {
  VisionSystemSim visionSim;
  TargetModel targetModel;
  SimCameraProperties cameraProperties;
  PhotonCamera camera;
  PhotonCameraSim cameraSim;
  int cameraId;

  public VisionIoSim() {
    
  }

  @Override
  public void updateInputs(VisionIoInputs inputs) {}
}
