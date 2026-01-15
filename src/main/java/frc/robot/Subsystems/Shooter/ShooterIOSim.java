package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim outTakeMotor1 =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1));
  private DCMotorSim outTakeMotor2 =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1));

  private double currentVoltage = 0;

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor1.setInputVoltage(voltage);
    outTakeMotor2.setInputVoltage(voltage);

    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = outTakeMotor1.getAngularPositionRad();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = outTakeMotor1.getAngularVelocityRPM();

    inputs1.position2 = outTakeMotor2.getAngularPositionRad();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = outTakeMotor2.getAngularVelocityRPM();
    outTakeMotor1.update(0.02);
    outTakeMotor2.update(0.02);
  }
}
