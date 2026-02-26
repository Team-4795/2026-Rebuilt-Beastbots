// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Shooter.ShooterIOReal;
import frc.robot.Subsystems.Shooter.ShooterIOSim;
// import frc.robot.Subsystems.climb.Climb;
// import frc.robot.Subsystems.climb.ClimbIOReal;
// import frc.robot.Subsystems.climb.ClimbIOSim;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIORedux;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSpark;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionConstants;
import frc.robot.Subsystems.vision.VisionIoSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoAlign;
import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Vision vision;
  private Shooter shooter;
  // private Climb climb;
  // private Intake intake;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. * */
  public RobotContainer() throws IOException {
    try {
      VisionConstants.aprilTagFieldLayout2 =
          AprilTagFieldLayout.loadFromResource(
              AprilTagFields.k2026RebuiltAndymark.m_resourceFile); // Placefolder
      VisionConstants.aprilTagFieldLayout2.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    } catch (IOException e) {
      e.printStackTrace();
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            Drive.createInstance(
                new GyroIORedux(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        // vision = Vision.createInstance(new VisionIoSim()); // probably causing memory problems
        // climb = Climb.Initialize(new ClimbIOReal());
        shooter = Shooter.Initialize(new ShooterIOReal());
        // intake = Intake.Initialize(new IntakeIOReal());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            Drive.createInstance(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = Vision.createInstance(new VisionIoSim());
        // climb = Climb.Initialize(new ClimbIOSim());
        shooter = Shooter.Initialize(new ShooterIOSim());
        // intake = Intake.Initialize(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            Drive.createInstance(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = Vision.createInstance(new VisionIoSim());
        // climb = Climb.Initialize(new ClimbIOSim());
        shooter = Shooter.Initialize(new ShooterIOSim());
        // intake = Intake.Initialize(new IntakeIOSim());
        break;
    }
    CanandEventLoop.getInstance();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));
    driverController
        .y()
        .whileTrue(
            Commands.parallel(
                DriveCommands.setRotationGoal(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> autoAlign.goalAngle),
                new autoAlign()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driverController
        .a()
        .whileTrue(
            Commands.run(
                () -> shooter.setGoal(3000, () -> driverController.y().getAsBoolean()), shooter));

    // COMMENTED OUT FOR TESTING PURPOSES
    // also there should not be two default commands for shooter
    // shooter.setDefaultCommand(
    //     Commands.run(() -> shooter.setGoal(0, () -> driverController.y().getAsBoolean()),
    // shooter));
    // shooter.setDefaultCommand(Commands.run(() -> shooter.setGoalSimple(0), shooter));
    shooter.setDefaultCommand(Commands.run(() -> shooter.setVoltageAll(0), shooter));
    // shooter.setDefaultCommand(Commands.run(() -> shooter.setShooterVoltage(0), shooter));

    // climb.setDefaultCommand(Commands.run(() -> climb.setVoltage(0), climb));

    // operatorController.leftTrigger().whileTrue(Commands.run(() -> climb.setVoltage(10)));
    // operatorController.rightTrigger().whileTrue(Commands.run(() -> climb.setVoltage(-10)));

    operatorController
        .rightBumper()
        .onTrue(
            Commands.run(
                () -> shooter.setShooterVoltage(ShooterConstants.shooterVoltage), shooter));
    operatorController.a().onTrue(Commands.run(() -> shooter.Configure(), shooter));

    // operatorController
    //     .leftBumper()
    //     .onTrue(Commands.run(() -> intake.setVoltage(10), intake))
    //     .onFalse(Commands.run(() -> intake.setVoltage(0), intake));

    // for testing use only

    // operatorController
    //     .leftBumper()
    //     .whileTrue(shooter.intake())
    //     .onFalse(Commands.run(() -> shooter.setVoltageAll(0), shooter));
    // operatorController
    //     .povDown()
    //     .onTrue(Commands.run(() -> shooter.setShooterVoltage(9), shooter))
    //     .onFalse(Commands.run(() -> shooter.setShooterVoltage(0), shooter));
    // operatorController
    //     .povUp()
    //     .onTrue(Commands.run(() -> shooter.setVoltage(-9), shooter))
    //     .onFalse(Commands.run(() -> shooter.setVoltage(0), shooter));
    operatorController.povUp().whileTrue(Commands.run(() -> shooter.setGoalSimple(3600)));
    operatorController.povLeft().whileTrue(Commands.run(() -> shooter.setGoalSimple(2000)));
    operatorController.povRight().whileTrue(Commands.run(() -> shooter.setGoalSimple(1000)));
    operatorController.povDown().whileTrue(Commands.run(() -> shooter.setGoalSimple(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
