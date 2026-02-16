// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOReal;
import frc.robot.Subsystems.Intake.IntakeIOSim;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Shooter.ShooterIOReal;
import frc.robot.Subsystems.Shooter.ShooterIOSim;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSpark;
import frc.robot.commands.AutoCommands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Shooter shooter;
  private Intake intake;
  private Drive drive;
  LoggedDashboardChooser<Command> autoChooser;
  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        shooter = Shooter.Initialize(new ShooterIOReal());
        intake = Intake.Initialize(new IntakeIOReal());
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
      case SIM:
        shooter = Shooter.Initialize(new ShooterIOSim());
        intake = Intake.Initialize(new IntakeIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
      default:
        shooter = Shooter.Initialize(new ShooterIOSim());
        intake = Intake.Initialize(new IntakeIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
    }

    // Configure the trigger bindings
    configureBindings();

    // Register commands
    NamedCommands.registerCommand("startShooter", AutoCommands.startShooter());
    NamedCommands.registerCommand("stopShooter", AutoCommands.stopShooter());
    NamedCommands.registerCommand("startIntake", AutoCommands.startIntake());
    NamedCommands.registerCommand("stopShooter", AutoCommands.stopIntake());
    NamedCommands.registerCommand("startClimb", AutoCommands.startClimb());

    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser", AutoBuilder.buildAutoChooser("Left Depot Climb"));
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
    driverController
        .rightBumper()
        .whileTrue(
            Commands.run(
                () -> shooter.setShooterVoltage(ShooterConstants.shooterVoltage), shooter));
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
