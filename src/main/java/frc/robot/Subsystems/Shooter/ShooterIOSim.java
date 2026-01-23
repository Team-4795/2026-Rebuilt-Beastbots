package frc.robot.Subsystems.Shooter;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOSim implements ShooterIO {
  // PID
  private final SparkMaxConfig config = new SparkMaxConfig();

  private DCMotorSim simMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.1, 2),
          DCMotor.getKrakenX60(2));

  private SimpleMotorFeedforward ffmodel =
      new SimpleMotorFeedforward(
          ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(5000, 5000);
  private final PIDController controller =
      new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);

  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private double currentVoltage = 0;
  private double setRPM = 0;

  @Override
  public void setGoal(double RPM) {
    setRPM = RPM;
  }

  @Override
  public void updateMotionProfile() {
    simMotor.setState(0, simMotor.getAngularVelocityRadPerSec());
    goal = new TrapezoidProfile.State(setRPM * ShooterConstants.voltsPerRPM, setRPM);
    setpoint = profile.calculate(0.02, setpoint, goal);
    double ffVoltage = ffmodel.calculate(setpoint.velocity * 2.0 * Math.PI / 60.0);

    setVoltage(
        ffVoltage
            + controller.calculate(
                simMotor.getAngularVelocityRadPerSec(), setpoint.velocity * 2.0 * Math.PI / 60.0));
  }

  @Override
  public void setVoltage(double voltage) {
    simMotor.setInputVoltage(voltage);

    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = simMotor.getAngularPositionRad();
    inputs1.position1 = simMotor.getAngularPositionRad();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = simMotor.getAngularVelocityRPM();

    inputs1.position2 = simMotor.getAngularPositionRad();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = simMotor.getAngularVelocityRPM();
    inputs1.velocity1 = simMotor.getAngularVelocityRPM();

    simMotor.update(0.02);
    simMotor.update(0.02);
    simMotor.update(0.02);
  }
}
