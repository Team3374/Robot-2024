package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.indexer.Indexer;

public class ShootDriveBack extends SequentialCommandGroup {
  //TODO: FIX
  public ShootDriveBack(Drive drive, Indexer indexer, Flywheel flywheel) {
    addCommands(
        // Commands.runOnce(() -> flywheel.runVelocity(3000), flywheel),
        new WaitCommand(2.5),
        Commands.parallel(
            // Commands.runOnce(() -> intake.runVelocity(-3000), intake),
            // Commands.runOnce(() -> intakeJoint.runPosition(20), intakeJoint),
            Commands.runOnce(() -> indexer.runVelocity(3000), indexer)),
        new WaitCommand(2.5),
        Commands.parallel(
            // Commands.runOnce(intake::stop, intake),
            // Commands.runOnce(() -> intakeJoint.runPosition(0), intakeJoint),
            Commands.runOnce(indexer::stop, indexer)),
            // Commands.runOnce(() -> flywheel.runVelocity(0), flywheel)),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-1.5, 0, 0)), drive),
        new WaitCommand(1),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive));
    // eli was not here.
  }
}
