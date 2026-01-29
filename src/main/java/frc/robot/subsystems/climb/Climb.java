package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    public ClimbIO io;  
 
      public static Intake instance;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    

      
    
    public void setVoltage(double volts) {
      io.setVoltage(volts);  
    }




    


    
}
