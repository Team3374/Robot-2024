// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunSingleClimber;
import frc.robot.commands.auto.DelayDriveBack;
import frc.robot.commands.auto.DriveBack;
import frc.robot.commands.auto.ShootDriveBack;
import frc.robot.commands.auto.ShootDriveLeft;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMaxCancoder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final Intake intake;
  //   private final IntakeJoint intakeJoint;
  private final Indexer indexer;
  private final Flywheel flywheel;

  private final Climber leftClimber;
  private final Climber rightClimber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controllerTwo = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> allianceChooser;
  //   private final LoggedDashboardNumber intakeSpeedInput =
  //       new LoggedDashboardNumber("Intake Speed", 3000.0);
  //   private final LoggedDashboardNumber intakeJointPositionInput =
  //       new LoggedDashboardNumber("Intake Joint Position", 20.5);
  private final LoggedDashboardNumber indexerSpeedInput =
      new LoggedDashboardNumber("Indexer Speed", 3000.0);
  private final LoggedDashboardNumber topflywheelSpeedInput =
      new LoggedDashboardNumber("topFlywheel Speed", 3000.0);
  private final LoggedDashboardNumber bottomflywheelSpeedInput =
      new LoggedDashboardNumber("bottomFlywheel Speed", 3000.0);
  private final LoggedDashboardNumber climberSpeedInput =
      new LoggedDashboardNumber("Climber Max Speed", 3000.0);
  private final LoggedDashboardNumber climberUpperLimit =
      new LoggedDashboardNumber("Climber Encoder Limit", 25);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMaxCancoder(0),
                new ModuleIOSparkMaxCancoder(1),
                new ModuleIOSparkMaxCancoder(2),
                new ModuleIOSparkMaxCancoder(3));
        // intake = new Intake(new IntakeIOTalonFX());
        // intakeJoint = new IntakeJoint(new IntakeJointIOSparkMax());

        indexer = new Indexer(new IndexerIOSparkMax());
        flywheel = new Flywheel(new FlywheelIOSparkMax(56), new FlywheelIOSparkMax(52));

        leftClimber = new Climber(new ClimberIOTalonFX(54));
        rightClimber = new Climber(new ClimberIOTalonFX(55));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // intake = new Intake(new IntakeIO() {});
        // intakeJoint = new IntakeJoint(new IntakeJointIO() {});
        indexer = new Indexer(new IndexerIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, new FlywheelIO() {});

        leftClimber = new Climber(new ClimberIO() {});
        rightClimber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // intake = new Intake(new IntakeIO() {});
        // intakeJoint = new IntakeJoint(new IntakeJointIO() {});
        indexer = new Indexer(new IndexerIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, new FlywheelIO() {});

        leftClimber = new Climber(new ClimberIO() {});
        rightClimber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive Back", new DriveBack(drive, 1));
    autoChooser.addOption("Delayed Drive Back", new DelayDriveBack(drive, 1));
    autoChooser.addOption("Shoot and Drive (Center)", new ShootDriveBack(drive, indexer, flywheel));
    autoChooser.addOption("Shoot and Drive (Left)", new ShootDriveLeft(drive, indexer, flywheel));
    // autoChooser.addOption(
    // "Shoot and Drive", new ShootDriveBack(drive, intake, intakeJoint, indexer, flywheel));
    autoChooser.addOption("Do Nothing", null);

    allianceChooser =
        new LoggedDashboardChooser<>("Alliance Selection", AutoBuilder.buildAutoChooser());

    allianceChooser.addOption(
        "Red", Commands.runOnce(() -> drive.setPose(new Pose2d(0, 0, new Rotation2d(135))), drive));
    allianceChooser.addOption(
        "Blue", Commands.runOnce(() -> drive.setPose(new Pose2d(0, 0, new Rotation2d(45))), drive));

    // Configure the button bindings
    configureButtonBindings();
    // intakeJoint.init();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // intakeJoint.setDefaultCommand(Commands.run(() -> intakeJoint.runPosition(0), intakeJoint));

    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> intakeJoint.runPosition(intakeJointPositionInput.get()),
    //             () -> intakeJoint.runPosition(0),
    //             intakeJoint))
    //     .whileTrue(new IntakeAutomated(intake, intakeSpeedInput.get()));
    // controller
    //     .leftBumper()
    //     // .whileTrue(
    //     //     Commands.startEnd(
    //     //         () -> intakeJoint.runPosition(intakeJointPositionInput.get()),
    //     //         () -> intakeJoint.runPosition(0),
    //     //         intakeJoint))
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> intake.runVelocity(intakeSpeedInput.get()), intake::stop, intake))
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> intakeJoint.runPosition(10), () -> intakeJoint.runPosition(0),
    // intakeJoint));
    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> intake.runVelocity(-intakeSpeedInput.get()), intake::stop, intake));
    // controller
    //     .b()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> intake.runVelocity(intakeSpeedInput.get()), intake::stop, intake));
    controllerTwo
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () ->
                    flywheel.runVelocity(
                        topflywheelSpeedInput.get(), bottomflywheelSpeedInput.get()),
                flywheel::stop,
                flywheel));
    //Add amp controls!
    controllerTwo
        .a()
        .whileTrue(
            Commands.startEnd(() -> flywheel.runVelocity(-750, -750), flywheel::stop, flywheel))
        .whileTrue(
            Commands.startEnd(
                () -> indexer.runVelocity(-indexerSpeedInput.get() * 0.5), indexer::stop, indexer));
    controllerTwo
        .leftBumper()
        // .whileTrue(
        //     Commands.startEnd(
        //         () -> intakeJoint.runPosition(intakeJointPositionInput.get()),
        //         intakeJoint::stop,
        //         intakeJoint))
        .whileTrue(
            Commands.startEnd(
                () -> indexer.runVelocity(indexerSpeedInput.get()), indexer::stop, indexer));
    // .whileTrue(
    //     Commands.startEnd(
    //         () -> intake.runVelocity(-intakeSpeedInput.get()), intake::stop, intake));
    controllerTwo
        .b()
        .whileTrue(
            Commands.startEnd(
                () -> indexer.runVelocity(-indexerSpeedInput.get()), indexer::stop, indexer));
    controller
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> indexer.runVelocity(indexerSpeedInput.get()), indexer::stop, indexer));
    controller
        .y()
        .whileTrue(
            Commands.startEnd(
                () -> indexer.runVelocity(-indexerSpeedInput.get()), indexer::stop, indexer));
    controllerTwo
        .povUp()
        .whileTrue(
            new RunClimber(
                leftClimber,
                rightClimber,
                climberSpeedInput.get(),
                () -> climberUpperLimit.get(),
                false));
    controllerTwo
        .povDown()
        .whileTrue(
            new RunClimber(
                leftClimber,
                rightClimber,
                climberSpeedInput.get(),
                () -> climberUpperLimit.get(),
                true));
    controllerTwo
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                () -> leftClimber.runVelocity(controllerTwo.getLeftY() * climberSpeedInput.get()),
                leftClimber::stop,
                leftClimber));
    controllerTwo
        .rightStick()
        .whileTrue(
            Commands.startEnd(
                () -> rightClimber.runVelocity(controllerTwo.getRightY() * climberSpeedInput.get()),
                rightClimber::stop,
                rightClimber));
    controllerTwo
        .x()
        .toggleOnTrue(
            new RunSingleClimber(
                leftClimber,
                rightClimber,
                climberSpeedInput.get(),
                () -> controllerTwo.getLeftY(),
                () -> controllerTwo.getRightY()));
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(0, 0, new Rotation2d(180)), 90), drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command getEndCommand() {
    return new RunSingleClimber(
        leftClimber, rightClimber, climberSpeedInput.get(), () -> -1, () -> -1);
  }

  public Command getAllianceCommand() {
    return allianceChooser.get();
  }
}
