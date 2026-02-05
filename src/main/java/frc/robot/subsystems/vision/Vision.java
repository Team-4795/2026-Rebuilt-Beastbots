package frc.robot.subsystems.vision;

public class Vision {
    private static Vision instance;
    private VisionIo currentIo;
    public static Vision getInstance()
    {
        return instance;
    }

    public static Vision createInstance(VisionIo io)
    {
        if (instance == null){
            return new Vision(io);
        }
        return instance;
    }

    public Vision(VisionIo io)
    {
        currentIo = io;
    }
    public void periodic(){
        io[i].updateInputs(inputs[i]);
        Logger.processInputs("Vision/" + VisionConstants.cameraIds[i], inputs[i]);
    }
}
