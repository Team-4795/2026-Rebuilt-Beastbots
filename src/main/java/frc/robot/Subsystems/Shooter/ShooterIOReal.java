package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOReal implements ShooterIO {
  private SparkMax outTakeMotor =
      new SparkMax(ShooterConstants.motorPort, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder outTakeEncoder = outTakeMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public ShooterIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(ShooterConstants.maxAmps);
    outTakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    outTakeMotor.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.position = outTakeEncoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = outTakeEncoder.getVelocity();
  }
}
