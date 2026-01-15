package frc.robot.Subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {
  private SparkMax outTakeMotor =
      new SparkMax(IntakeConstants.motorPort, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder outTakeEncoder = outTakeMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public IntakeIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(IntakeConstants.maxAmps);
    outTakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    outTakeMotor.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = outTakeEncoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = outTakeEncoder.getVelocity();
  }
}
