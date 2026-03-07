package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static Shooter instance;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private LoggedTunableNumber kp = new LoggedTunableNumber("Shooter1/Kp", ShooterConstants.PID.kP);
  private LoggedTunableNumber ki = new LoggedTunableNumber("Shooter1/Ki", ShooterConstants.PID.kI);
  private LoggedTunableNumber kd = new LoggedTunableNumber("Shooter1/Kd", ShooterConstants.PID.kD);

  private LoggedTunableNumber ks = new LoggedTunableNumber("Shooter1/Ks", ShooterConstants.PID.kS);
  private LoggedTunableNumber kv = new LoggedTunableNumber("Shooter1/Kv", ShooterConstants.PID.kV);
  private LoggedTunableNumber ka = new LoggedTunableNumber("Shooter1/Ka", ShooterConstants.PID.kA);

  private LoggedTunableNumber kp2 =
      new LoggedTunableNumber("Shooter2/Kp2", ShooterConstants.PID.kP2);
  private LoggedTunableNumber ki2 =
      new LoggedTunableNumber("Shooter2/Ki2", ShooterConstants.PID.kI2);
  private LoggedTunableNumber kd2 =
      new LoggedTunableNumber("Shooter2/Kd2", ShooterConstants.PID.kD2);

  private LoggedTunableNumber ks2 =
      new LoggedTunableNumber("Shooter2/Ks2", ShooterConstants.PID.kS2);
  private LoggedTunableNumber kv2 =
      new LoggedTunableNumber("Shooter2/Kv2", ShooterConstants.PID.kV2);
  private LoggedTunableNumber ka2 =
      new LoggedTunableNumber("Shooter2/Ka2", ShooterConstants.PID.kA2);
  private LoggedTunableNumber RPM = new LoggedTunableNumber("Shooter/RPM", 5000);
  public ShooterIO shooterIo;
  // private Drive drive;

  private double goalRPM = RPM.getAsDouble();

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

  public void setGoalStatic() {
    double targetRPM = RPM.getAsDouble();
    goalRPM = targetRPM;
    // if (isYHeldDown.getAsBoolean() && targetRPM != 0) {
    //   Translation2d currentGoal = autoAlign.lookGoal;
    //   Translation2d drivePos = drive.getPose().getTranslation();
    //   double Jello =
    //       Math.sqrt(
    //           Math.pow(currentGoal.getX() - drivePos.getX(), 2)
    //               + Math.pow(currentGoal.getY() - drivePos.getY(), 2));

    //   if (Jello > ShooterConstants.minDistance && Jello < ShooterConstants.maxDistance) {
    //     targetRPM = ShooterConstants.distanceFunction(Jello);
    //   }
    // }
    this.readyToShoot(5000);
  }

  public void setGoalDynamic(double distance) {
    double rpm = ShooterConstants.shooterMap.get(distance);
    goalRPM = rpm;
    this.readyToShoot(rpm);
  }

  public void readyToShoot(double targetRPM) {
    shooterIo.setGoal(targetRPM);
    if (Math.abs(inputs.velocity1 - targetRPM) < ShooterConstants.tolerableRpmRangeShooter
        && targetRPM != 0) { // idc if we overshoot, as long as we get the indexer
      // spinning
      setIndexerVoltage(ShooterConstants.indexerVoltage);
    }
  }

  public void forceShoot() {
    shooterIo.setGoal(goalRPM);
    shooterIo.setIndexerVoltage(ShooterConstants.indexerVoltage);
  }

  public void setGoal(double rpm) {
    Logger.recordOutput("Outake test rpm", rpm);
    goalRPM = rpm;
    shooterIo.setGoal(rpm);
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

  public void defaultCommand() {
    shooterIo.setIndexerVoltage(0);
    shooterIo.setGoal(0);
  }

  public void intake() {
    shooterIo.setIndexerVoltage(-ShooterConstants.indexerVoltage);
    shooterIo.setGoal(2000);
  }

  public void revShooter() {
    shooterIo.setGoal(RPM.getAsDouble());
  }

  public Shooter(ShooterIO io) {
    shooterIo = io;
    // drive = Drive.getInstance();
  }

  @Override
  public void periodic() {
    shooterIo.updateInputs(inputs);
    shooterIo.updateMotionProfile();
    Logger.processInputs("Shooter/Shooter", inputs);
  }
}
