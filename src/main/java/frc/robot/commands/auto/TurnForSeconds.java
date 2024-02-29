package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class TurnForSeconds extends Command {

  private static LoggedTunableNumber speed = new LoggedTunableNumber("DriveForward/Speed");

  private final Drive drive;
  private final Timer timer = new Timer();

  private double driveForSeconds;

  /** Drives straight forward. */
  public TurnForSeconds(Drive drive, double numberOfSeconds) {
    addRequirements(drive);
    this.drive = drive;
    this.driveForSeconds = numberOfSeconds;

    speed.setDefault(1);
  }

  // Called when the command is initially schedule.
  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed.get()));
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(this.driveForSeconds);
  }
}
