// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Hopper.*;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOReal;
import frc.robot.Subsystems.Shooter.ShooterIOSim;
import frc.robot.Subsystems.climb.Climb;
import frc.robot.Subsystems.climb.ClimbIOSim;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIORedux;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSpark;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIoReal;
import frc.robot.Subsystems.vision.VisionIoSim;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  LoggedDashboardChooser<Command> autoChooser;
  private Vision vision;
  private Shooter shooter;
  private Climb climb;
  // private Intake intake;
  private Hopper hopper;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. * */
  public RobotContainer() throws IOException {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (Constants.shouldDrive) {
          drive =
              Drive.createInstance(
                  new GyroIORedux(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));
        } else {
          drive =
              Drive.createInstance(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
        }
        shooter = Shooter.Initialize(new ShooterIOReal());
        hopper = Hopper.Initialize(new HopperIOSim());
        vision = Vision.createInstance(new VisionIoReal(0));
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
        // vision = Vision.createInstance(new VisionIoSim());
        climb = Climb.Initialize(new ClimbIOSim());
        shooter = Shooter.Initialize(new ShooterIOSim());
        hopper = Hopper.Initialize(new HopperIOSim());
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
        climb = Climb.Initialize(new ClimbIOSim());
        shooter = Shooter.Initialize(new ShooterIOSim());
        hopper = Hopper.Initialize(new HopperIOSim());
        // intake = Intake.Initialize(new IntakeIOSim());
        break;
    }
    CanandEventLoop.getInstance();
    configureBindings();

    NamedCommands.registerCommand("startShooter", AutoCommands.startShooter());
    NamedCommands.registerCommand("stopShooter", AutoCommands.stopShooter());
    NamedCommands.registerCommand("startIntake", AutoCommands.startIntake());
    NamedCommands.registerCommand("stopIntake", AutoCommands.stopIntake());
    NamedCommands.registerCommand("startClimb", AutoCommands.startClimb());
    NamedCommands.registerCommand("shootDynamic", AutoCommands.shootDynamic());
    NamedCommands.registerCommand("shootDynamicForever", AutoCommands.shootDynamicForever());

    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser", AutoBuilder.buildAutoChooser("Top Depot Climb"));
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
            () -> -driverController.getLeftY() * 0.5,
            () -> -driverController.getLeftX() * 0.5,
            () -> -driverController.getRightX() * 0.5));

    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> Rotation2d.kZero));

    // auto align i think
    //
    // driverController
    //     .y()
    //     .whileTrue(
    //         Commands.parallel(
    //             DriveCommands.setRotationGoal(
    //                 drive,
    //                 () -> driverController.getLeftY(),
    //                 () -> driverController.getLeftX(),
    //                 () -> autoAlign.goalAngle),
    //             new autoAlign()));
    // operatorController
    //     .povUp()
    //     .whileTrue(
    //         Commands.parallel(
    //             DriveCommands.setRotationGoal(
    //                 drive,
    //                 () -> driverController.getLeftY(),
    //                 () -> driverController.getLeftX(),
    //                 () -> autoAlign.goalAngle),
    //             new autoAlign()));

    // default commands
    shooter.setDefaultCommand(Commands.run(() -> shooter.defaultCommand(), shooter));
    // climb.setDefaultCommand(Commands.run(() -> climb.setVoltage(0), climb));
    hopper.setDefaultCommand(Commands.run(() -> hopper.setExtended(true), hopper));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when X button is pressed
    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // driverController
    //     .povDown()
    //     .onTrue(Commands.runOnce(() -> drive.toggleDriveSensitivity(), drive));

    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -driverController.getLeftY() / 3.0,
    //             () -> -driverController.getLeftX() / 3.0,
    //             () -> -driverController.getRightX()));

    // climb no more
    // operatorController.leftTrigger().whileTrue(Commands.run(() -> climb.setVoltage(6), climb));
    // operatorController.rightTrigger().whileTrue(Commands.run(() -> climb.setVoltage(-6), climb));

    // hopper extension
    // operatorController
    //     .leftTrigger()
    //     .whileTrue(Commands.run(() -> hopper.setExtended(true), hopper));
    // operatorController
    //     .rightTrigger()
    //     .whileTrue(Commands.run(() -> hopper.setExtended(false), hopper));

    // shooter
    // operatorController
    //     .leftBumper()
    //     .whileTrue(AutoCommands.shootDynamic()); // most likely doesnt work
    // driverController.b().whileTrue(Commands.run(() -> shooter.forceShoot(), shooter));
    driverController
        .rightTrigger()
        .whileTrue(Commands.run(() -> shooter.revShooter(), shooter)); // spins up shooter
    // driverController.rightBumper().whileTrue(Commands.run(() -> shooter.setGoalStatic(),
    // shooter));
    // operatorController.a().whileTrue(Commands.run(() -> shooter.setGoalStatic(), shooter));
    driverController.rightBumper().whileTrue(Commands.run(() -> shooter.runIndexer(5), shooter)); // shoots

    driverController.leftBumper().whileTrue(Commands.run(() -> shooter.intake(), shooter)); // intake

    // driverController.rightTrigger().whileTrue(AutoCommands.shootDynamic());
    // driverController.leftTrigger().whileTrue(Commands.run(() -> shooter.revShooter(), shooter));

    // operatorController.povLeft().whileTrue(Commands.run(() -> shooter.unstuck(), shooter));

    // operatorController.povDown().onTrue(Commands.runOnce(() -> shooter.Configure(), shooter));

    // operatorController
    //     .povUp()
    //     .whileTrue(Commands.run(() -> drive.sysIdDynamic(Direction.kForward)));
    // operatorController
    //     .povDown()
    //     .whileTrue(Commands.run(() -> drive.sysIdDynamic(Direction.kReverse)));
    // operatorController
    //     .povLeft()
    //     .whileTrue(Commands.run(() -> drive.sysIdQuasistatic(Direction.kForward)));
    // operatorController
    //     .povRight()
    //     .whileTrue(Commands.run(() -> drive.sysIdQuasistatic(Direction.kReverse)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }
}
