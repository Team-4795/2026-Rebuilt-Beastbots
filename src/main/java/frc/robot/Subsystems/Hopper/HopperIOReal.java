package frc.robot.Subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOReal implements HopperIO {
  private SparkMax motorL =
      new SparkMax(HopperConstants.canIdL, SparkLowLevel.MotorType.kBrushless);
  private SparkMax motorR =
      new SparkMax(HopperConstants.canIdR, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder encoder = motorR.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public HopperIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(HopperConstants.maxAmps);
    motorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorL.clearFaults();
    config.follow(HopperConstants.canIdL, true);
    motorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    motorL.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = encoder.getVelocity();
  }
}
