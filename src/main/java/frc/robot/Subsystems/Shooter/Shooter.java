package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.commands.autoAlign;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static Shooter instance;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
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

  private double distanceFunction(double x) {
    //RPM = distance^2+4

    double distanceRPM = x*x+4;
    return distanceRPM;
    
  }

  public void setGoal(double defaultRPM, BooleanSupplier isYHeldDown) {
    double targetRPM = defaultRPM;
    if (isYHeldDown.getAsBoolean() && defaultRPM != 0)
    {
      Translation2d currentGoal = autoAlign.lookGoal;
      Translation2d drivePos = drive.getPose().getTranslation();
      double Jello = Math.sqrt(Math.pow(currentGoal.getX()-drivePos.getX(),2)+Math.pow(currentGoal.getY()-drivePos.getY(),2));

      if (Jello > ShooterConstants.minDistance && Jello < ShooterConstants.maxDistance)
      {
        targetRPM = distanceFunction(Jello);
      }
      
    }
    shooterIo.setGoal(targetRPM);
  }

  public void setShooterVoltage(double volts) {
    shooterIo.setVoltage(volts);
  }

  public Shooter(ShooterIO io) {
    shooterIo = io;
    drive = Drive.getInstance();
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    Logger.processInputs("Shooter/Shooter", inputs);
  }
}
