package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  public ClimbIO io = new ClimbIOReal();

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
