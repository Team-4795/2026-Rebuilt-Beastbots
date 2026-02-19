package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIoSim implements VisionIo {
  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCamera cameraOne;
  private PhotonCameraSim cameraSimOne;

  public VisionIoSim() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout2);

      cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(78));
      cameraProp.setCalibError(0.38, 0.2);
      cameraProp.setFPS(30);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      // Add all three cameras to sim
      cameraOne = new PhotonCamera(VisionConstants.cameraNames[0]);
      cameraSimOne = new PhotonCameraSim(cameraOne, cameraProp);
      cameraSimOne.setMaxSightRange(8);

      visionSim.addCamera(cameraSimOne, VisionConstants.positionOfCameras[0]);
    }
  }

  @Override
  public void updateInputs(VisionIoInputs inputs) {
    visionSim.update(Drive.getInstance().getPose());
    cameraSimOne.enableRawStream(true);
    cameraSimOne.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSimOne.enableDrawWireframe(true);
    Logger.recordOutput(
        "Vision/Camera Poses/Front Cam Pose", visionSim.getCameraPose(cameraSimOne).get());
    Logger.recordOutput("number", 5);
  }
}
