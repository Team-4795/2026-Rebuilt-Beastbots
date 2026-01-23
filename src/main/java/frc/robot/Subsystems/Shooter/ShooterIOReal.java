package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// Gear ratio intake 1.777777777

public class ShooterIOReal implements ShooterIO {
  private SparkMax outTakeMotor1 =
      new SparkMax(ShooterConstants.motorPort1, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  // Left Motor
  private RelativeEncoder outTakeEncoder1 = outTakeMotor1.getEncoder();

  // PID
  SparkClosedLoopController m_controller = outTakeMotor1.getClosedLoopController();

  private final SparkMaxConfig config = new SparkMaxConfig();
  private double RPMgoal = 0;

  @Override
  public void setGoal(double RPM) {
    if (RPM != RPMgoal) {
      RPMgoal = RPM;
    }
  }

  @Override
  public void updateMotionProfile() {
    m_controller.setSetpoint(RPMgoal, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public ShooterIOReal() {

    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(ShooterConstants.maxAmps);
    config
        .closedLoop
        .p(ShooterConstants.PID.kP, ClosedLoopSlot.kSlot1)
        .i(ShooterConstants.PID.kI, ClosedLoopSlot.kSlot1)
        .d(ShooterConstants.PID.kD, ClosedLoopSlot.kSlot1)
        .outputRange(0, ShooterConstants.kMaxOutput);
    config
        .closedLoop
        .feedForward
        .kS(ShooterConstants.PID.kS)
        .kV(ShooterConstants.PID.kV)
        .kG(ShooterConstants.PID.kG);

    // .kG(g) // kG is a linear gravity feedforward, for an elevator
    // .kCos(g) // kCos is a cosine gravity feedforward, for an arm
    // .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position

    config
        .closedLoop
        .maxMotion
        .maxAcceleration(ShooterConstants.maxAcceleration)
        .allowedProfileError(ShooterConstants.allowedErr);
    outTakeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    outTakeMotor1.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor1.setVoltage(voltage);

    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = outTakeEncoder1.getPosition();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = outTakeEncoder1.getVelocity();

    inputs1.position2 = outTakeEncoder1.getPosition();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = outTakeEncoder1.getVelocity();
  }
}
