package frc.robot.Subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOReal implements HopperIO {
  private SparkMax motor =
      new SparkMax(HopperConstants.motorPort, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public HopperIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(HopperConstants.maxAmps);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = encoder.getVelocity();
  }
}
