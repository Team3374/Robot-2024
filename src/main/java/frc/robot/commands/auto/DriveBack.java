package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveBack extends SequentialCommandGroup {
  public DriveBack(Drive drive) {
    addCommands(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, -12, 0)), drive),
        new WaitCommand(5),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive)
    );
  }
}
