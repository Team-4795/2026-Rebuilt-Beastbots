package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim outTakeMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2), DCMotor.getKrakenX60(1));
  private double currentVoltage = 0;


  @Override
  public void setVoltage(double voltage) {
    outTakeMotor.setInputVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.position = outTakeMotor.getAngularPositionRad();
    inputs.voltage = currentVoltage;
    inputs.velocity = outTakeMotor.getAngularVelocityRPM();
  }
}
