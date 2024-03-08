package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DelayDriveBack extends SequentialCommandGroup {

  public DelayDriveBack(Drive drive, double driveTime) {
    addCommands(
        new WaitCommand(3),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-1.5, 0, 0)), drive),
        new WaitCommand(driveTime),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive));
  }
}
