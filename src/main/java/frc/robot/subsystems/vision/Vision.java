package frc.robot.subsystems.vision;

import java.lang.System.Logger;

import frc.robot.subsystems.vision.VisionIo.VisionIoInputs;

public class Vision {
    private static Vision instance;
    private VisionIo currentIo;
    private VisionIoInputs inputs;

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
        currentIo.updateInputs(inputs);
        Logger.processInputs(inputs);

        
    }
}
