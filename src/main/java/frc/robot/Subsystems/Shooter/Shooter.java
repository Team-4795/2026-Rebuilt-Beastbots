package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.commands.autoAlign;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static Shooter instance;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private LoggedTunableNumber kp = new LoggedTunableNumber("Kp", ShooterConstants.PID.kP);
  private LoggedTunableNumber ki = new LoggedTunableNumber("Ki", ShooterConstants.PID.kI);
  private LoggedTunableNumber kd = new LoggedTunableNumber("Kd", ShooterConstants.PID.kD);

  private LoggedTunableNumber ks = new LoggedTunableNumber("Kp", ShooterConstants.PID.kP);
  private LoggedTunableNumber kv = new LoggedTunableNumber("Ki", ShooterConstants.PID.kI);
  private LoggedTunableNumber ka = new LoggedTunableNumber("Kd", ShooterConstants.PID.kD);

  private LoggedTunableNumber kp2 = new LoggedTunableNumber("Kp2", ShooterConstants.PID.kP2);
  private LoggedTunableNumber ki2 = new LoggedTunableNumber("Ki2", ShooterConstants.PID.kI2);
  private LoggedTunableNumber kd2 = new LoggedTunableNumber("Kd2", ShooterConstants.PID.kD2);

  private LoggedTunableNumber ks2 = new LoggedTunableNumber("Kp2", ShooterConstants.PID.kP2);
  private LoggedTunableNumber kv2 = new LoggedTunableNumber("Ki2", ShooterConstants.PID.kI2);
  private LoggedTunableNumber ka2 = new LoggedTunableNumber("Kd2", ShooterConstants.PID.kD2);

  public ShooterIO shooterIo;
  private Drive drive;

  public static Shooter getInstance() {
    return instance;
  }

  public static Shooter Initialize(ShooterIO shooterIo) {
    if (instance == null) {
      instance = new Shooter(shooterIo);
    }
    return instance;
  }

  public void Configure() {
    shooterIo.configure1(
        kp.getAsDouble(),
        ki.getAsDouble(),
        kd.getAsDouble(),
        ks.getAsDouble(),
        kv.getAsDouble(),
        ka.getAsDouble());
    shooterIo.configure2(
        kp2.getAsDouble(),
        ki2.getAsDouble(),
        kd2.getAsDouble(),
        ks2.getAsDouble(),
        kv2.getAsDouble(),
        ka2.getAsDouble());
  }

  public void setGoal(double defaultRPM, BooleanSupplier isYHeldDown) {
    double targetRPM = defaultRPM;
    if (isYHeldDown.getAsBoolean() && defaultRPM != 0) {
      Translation2d currentGoal = autoAlign.lookGoal;
      Translation2d drivePos = drive.getPose().getTranslation();
      double Jello =
          Math.sqrt(
              Math.pow(currentGoal.getX() - drivePos.getX(), 2)
                  + Math.pow(currentGoal.getY() - drivePos.getY(), 2));

      if (Jello > ShooterConstants.minDistance && Jello < ShooterConstants.maxDistance) {
        targetRPM = ShooterConstants.distanceFunction(Jello);
      }
    }
    shooterIo.setGoal(targetRPM);
  }

  public void setGoalSimple(double rpm) {
    shooterIo.setGoal(rpm);
  }

  public void shootWithRPM(double rpm) {
    setGoalSimple(rpm);
    setShooterVoltage(9);
  }

  public void setShooterVoltage(double volts) {
    shooterIo.setShooterVoltage(volts);
  }

  public void setVoltage(double volts) {
    shooterIo.setVoltage(volts);
  }

  public void setVoltageAll(double volts) {
    shooterIo.setVoltageAll(volts);
  }

  public Command intake() {
    return Commands.parallel(
        Commands.run(() -> shooterIo.setVoltage(5), this),
        Commands.run(() -> shooterIo.setShooterVoltage(-3)));
  }

  public Shooter(ShooterIO io) {
    shooterIo = io;
    drive = Drive.getInstance();
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    shooterIo.updateMotionProfile();
    Logger.processInputs("Shooter/Shooter", inputs);
  }
}
