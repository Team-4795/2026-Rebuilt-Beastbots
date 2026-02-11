package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), ClimbConstants.moi, ClimbConstants.gearing),
          DCMotor.getNEO(1));

  @Override
  public void setVoltage(double volts) {
    motor.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.velocity = motor.getAngularVelocityRPM();
    inputs.voltage = motor.getInputVoltage();
    inputs.current = motor.getCurrentDrawAmps();
    inputs.position = motor.getAngularPositionRad();
    motor.update(0.02);
  }
}
