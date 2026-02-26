package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim outTakeMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(2), 0.1, 1.5), DCMotor.getNeo550(2));

  private SimpleMotorFeedforward ffmodel =
      new SimpleMotorFeedforward(
          ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
  private final PIDController controller =
      new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
  private double currentVoltage = 0;
  private double setRPM = 0;

  @Override
  public void setGoal(double RPM) {
    setRPM = RPM;
  }

  @Override
  public void updateMotionProfile() {
    outTakeMotor.setState(0, outTakeMotor.getAngularVelocityRadPerSec());
    double ffVoltage = ffmodel.calculate(setRPM);

    setVoltage(
        MathUtil.clamp(
            ffVoltage + controller.calculate(outTakeMotor.getAngularVelocityRPM(), setRPM),
            -12,
            12));
  }

  @Override
  public void setVoltage(double voltage) {
    outTakeMotor.setInputVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs1) {
    inputs1.position1 = outTakeMotor.getAngularPositionRad();
    inputs1.position1 = outTakeMotor.getAngularPositionRad();
    inputs1.voltage1 = currentVoltage;
    inputs1.velocity1 = outTakeMotor.getAngularVelocityRPM();

    inputs1.position2 = outTakeMotor.getAngularPositionRad();
    inputs1.voltage2 = currentVoltage;
    inputs1.velocity2 = outTakeMotor.getAngularVelocityRPM();
    inputs1.velocity1 = outTakeMotor.getAngularVelocityRPM();

    outTakeMotor.update(0.02);

    updateMotionProfile();
  }
}
