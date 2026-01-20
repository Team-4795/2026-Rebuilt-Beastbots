package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOReal implements ShooterIO {
  // PID
  private SimpleMotorFeedforward ffmodel =
      new SimpleMotorFeedforward(
          ShooterConstants.PID.kS, ShooterConstants.PID.kG, ShooterConstants.PID.kG);
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ShooterConstants.maxVeloxity, ShooterConstants.maxAcceleration);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private PIDController controller =
      new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);

  private SparkMax outTakeMotor1 =
      new SparkMax(ShooterConstants.motorPort1, SparkLowLevel.MotorType.kBrushless);
  private SparkMax outTakeMotor2 =
      new SparkMax(ShooterConstants.motorPort2, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  // Left Motor
  private RelativeEncoder outTakeEncoder1 = outTakeMotor1.getEncoder();
  // Right Motor
  private RelativeEncoder outTakeEncoder2 = outTakeMotor2.getEncoder();

  private final SparkMaxConfig config = new SparkMaxConfig();

  @Override
  public void setGoal(double RPM) {
    if (RPM != goal.velocity) {
      setpoint =
          new TrapezoidProfile.State(outTakeEncoder1.getPosition(), outTakeEncoder2.getVelocity());
      goal = new TrapezoidProfile.State(0, RPM);
    }
  }

  @Override
  public void updateMotionProfile() {
    // double prevVelocity = setpoint.velocity;

    setpoint = profile.calculate(0.02, setpoint, goal);
    // double acceleration = (setpoint.velocity - prevVelocity) / 0.02;
    double ffvolts = ffmodel.calculate(setpoint.velocity);
    double pidvolts = controller.calculate(outTakeEncoder2.getPosition(), setpoint.position);
    setVoltage(ffvolts + pidvolts);
  }

  public ShooterIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(ShooterConstants.maxAmps);
    outTakeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    outTakeMotor1.clearFaults();
    outTakeMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    outTakeMotor2.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor1.setVoltage(voltage);
    outTakeMotor2.setVoltage(voltage);

    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = outTakeEncoder1.getPosition();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = outTakeEncoder1.getVelocity();

    inputs1.position2 = outTakeEncoder2.getPosition();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = outTakeEncoder2.getVelocity();
  }
}
