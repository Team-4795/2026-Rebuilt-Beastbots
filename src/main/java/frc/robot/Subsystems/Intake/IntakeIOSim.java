package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1));
  private double currentVoltage = 0;

  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(
    IntakeConstants.SIM.KS,
    IntakeConstants.SIM.KG,
    IntakeConstants.SIM.KV
  );

  private final TrapezoidProfile.Constraints constraints = 
    new TrapezoidProfile.Constraints(
      IntakeConstants.SIM.MAX_VELOCITY, 
      IntakeConstants.SIM.MAX_ACCELERATION
    );

  private final PIDController controller = new PIDController(
    IntakeConstants.SIM.KP,
    IntakeConstants.SIM.KI,
    IntakeConstants.SIM.KD
  );

  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateMotionProfile() {
      setpoint = profile.calculate(0.02, setpoint, goal);
      setVoltage(
        ffmodel.calculateWithVelocities(motor.getAngularVelocityRPM(), setpoint.velocity)
        + controller.calculate(motor.getAngularPositionRotations(), setpoint.position
      ));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = motor.getAngularPositionRotations();
    inputs.voltage = currentVoltage;
    inputs.velocity = motor.getAngularVelocityRPM();
  }
}
