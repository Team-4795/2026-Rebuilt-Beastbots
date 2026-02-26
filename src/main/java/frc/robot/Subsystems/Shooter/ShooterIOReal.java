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
  private SparkMax outTakeMotor2 =
      new SparkMax(ShooterConstants.motorPort2, SparkLowLevel.MotorType.kBrushless);

  // shooter only
  private SparkMax outTakeMotor3 =
      new SparkMax(ShooterConstants.motorPort3, SparkLowLevel.MotorType.kBrushless);

  private double currentVoltage = 0;
  private double currentVoltage3 = 0;

  // Left Motor
  private RelativeEncoder outTakeEncoder1 = outTakeMotor1.getEncoder();
  private RelativeEncoder outTakeEncoder2 = outTakeMotor2.getEncoder();
  private RelativeEncoder outTakeEncoder3 = outTakeMotor3.getEncoder();

  // PID
  SparkClosedLoopController m_controller = outTakeMotor1.getClosedLoopController();
  SparkClosedLoopController m_controller2 = outTakeMotor2.getClosedLoopController();
  // private PIDController controller1 = new PIDController(ShooterConstants.PID.kP,
  // ShooterConstants.PID.kI, ShooterConstants.PID.kD);
  // private PIDController controller2 = new PIDController(ShooterConstants.PID.kP2,
  // ShooterConstants.PID.kI2, ShooterConstants.PID.kD2)

  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkMaxConfig config2 = new SparkMaxConfig();
  private double RPMgoal = 0;

  @Override
  public void configure1(double kp, double ki, double kd, double ks, double kv, double ka) {
    config.closedLoop.pid(kp, ki, kd).outputRange(0, ShooterConstants.kMaxOutput);
    config.closedLoop.feedForward.kS(ks).kV(kv).kA(ka);
    outTakeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configure2(double kp, double ki, double kd, double ks, double kv, double ka) {
    config2.closedLoop.pid(kp, ki, kd).outputRange(0, ShooterConstants.kMaxOutput);
    config2.closedLoop.feedForward.kS(ks).kV(kv).kA(ka);
    outTakeMotor3.configure(
        config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setGoal(double RPM) {
    if (RPM != RPMgoal) {
      RPMgoal = RPM;
    }
  }

  @Override
  public void updateMotionProfile() {
    m_controller.setSetpoint(RPMgoal, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_controller2.setSetpoint(RPMgoal, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public ShooterIOReal() {

    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(ShooterConstants.maxAmps);

    // .kG(g) // kG is a linear gravity feedforward, for an elevator
    // .kCos(g) // kCos is a cosine gravity feedforward, for an arm
    // .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position

    config
        .closedLoop
        .maxMotion
        .maxAcceleration(ShooterConstants.maxAcceleration)
        .allowedProfileError(ShooterConstants.allowedErr);
    config.smartCurrentLimit(ShooterConstants.currentLimit);
    config.closedLoop.pid(
        ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
    config
        .closedLoop
        .feedForward
        .kS(ShooterConstants.PID.kS)
        .kV(ShooterConstants.PID.kV)
        .kA(ShooterConstants.PID.kA);
    outTakeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2.apply(config);
    config2
        .closedLoop
        .pid(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD)
        .outputRange(0, ShooterConstants.kMaxOutput);
    config2
        .closedLoop
        .feedForward
        .kS(ShooterConstants.PID.kS2)
        .kV(ShooterConstants.PID.kV2)
        .kA(ShooterConstants.PID.kA2);

    // probably need different pid constants
    outTakeMotor3.configure(
        config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(ShooterConstants.motorPort1, true); // invert?
    outTakeMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    outTakeMotor1.clearFaults();
    outTakeMotor2.clearFaults();
    outTakeMotor3.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor1.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void setVoltageAll(double voltage) {
    outTakeMotor1.setVoltage(voltage);
    outTakeMotor3.setVoltage(voltage);
    currentVoltage = voltage;
    currentVoltage3 = voltage;
  }

  @Override
  public void setShooterVoltage(double voltage) {
    outTakeMotor3.setVoltage(voltage);
    currentVoltage3 = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = outTakeEncoder1.getPosition();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = outTakeEncoder1.getVelocity();

    inputs1.position2 = outTakeEncoder2.getPosition();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = outTakeEncoder2.getVelocity();

    inputs1.position3 = outTakeEncoder3.getPosition();
    inputs1.voltage3 = currentVoltage3;
    inputs1.velocity3 = outTakeEncoder3.getVelocity();

    inputs1.goalVelocity = RPMgoal;
    inputs1.setpointVelocity = m_controller.getSetpoint();
  }
}
