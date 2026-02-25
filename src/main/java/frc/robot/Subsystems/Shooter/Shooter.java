package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.commands.autoAlign;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static Shooter instance;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public static final PIDController controller = new PIDController(1.3, 0.5, 0.0);

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
    setIndexerVoltage(9);
  }

  public void setIndexerVoltage(double volts) {
    shooterIo.setIndexerVoltage(volts);
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
        Commands.run(() -> shooterIo.setIndexerVoltage(-3)));
  }

  public Shooter(ShooterIO io) {
    shooterIo = io;
    drive = Drive.getInstance();
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    shooterIo.updateMotionProfile();
    Logger.recordOutput("hi3p4", 5);
    Logger.processInputs("Shooter/Shooter", inputs);
  }
}
