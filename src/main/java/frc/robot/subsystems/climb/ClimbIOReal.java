package frc.robot.Subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOReal implements ClimbIO {
  private SparkMax ClimbMotor =
      new SparkMax(ClimbConstants.motorPort, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder encoder = ClimbMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();
  double volts = 0;

  public ClimbIOReal() {
    config.smartCurrentLimit(ClimbConstants.CurrentLimit);
    config.idleMode(IdleMode.kBrake);
    ClimbMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    volts = voltage;
    ClimbMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.voltage = volts;
    inputs.current = ClimbMotor.getOutputCurrent();
    inputs.velocity = ClimbMotor.getEncoder().getVelocity();
  }
}
