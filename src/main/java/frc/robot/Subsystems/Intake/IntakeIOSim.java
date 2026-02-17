package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1));
  private double currentVoltage = 0;

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = motor.getAngularPositionRotations();
    inputs.voltage = currentVoltage;
    inputs.velocity = motor.getAngularVelocityRPM();
  }
}
