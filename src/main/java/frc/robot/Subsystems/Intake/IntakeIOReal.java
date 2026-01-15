package frc.robot.Subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IntakeIOReal implements IntakeIO {
  private SparkMax motor =
      new SparkMax(IntakeConstants.motorPort, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(
    IntakeConstants.KS,
    IntakeConstants.KG,
    IntakeConstants.KV
  );
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
    IntakeConstants.MAX_VELOCITY, 
    IntakeConstants.MAX_ACCELERATION
  );
  private PIDController controller = new PIDController(KP.get(), KI.get(), KD.get());
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  public double pidVolts; 
  public double feedForwardVolts; 

  public IntakeIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(IntakeConstants.maxAmps);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.clearFaults();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void setGoal(double setpoint) {
    motor.set(pid.calculate(encoder.getPosition(), setpoint));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = encoder.getVelocity();
  }
}
