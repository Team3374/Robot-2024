package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class RunClimber extends Command {
  private Climber leftClimber;
  private Climber rightClimber;

  private double velocity;

  private boolean reversed;

  /** Creates a new ClimberToPosition */
  public RunClimber(Climber leftClimber, Climber rightClimber, double velocity, boolean reversed) {
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;

    this.velocity = velocity;

    this.reversed = reversed;

    addRequirements(leftClimber, rightClimber);
  }

  /** Called when the command is initially scheduled */
  @Override
  public void initialize() {
    leftClimber.stop();
    rightClimber.stop();

    leftClimber.softLimitEnabled(true);
    rightClimber.softLimitEnabled(true);
  }

  /** Called every time the scheduler runs while the command is scheduled */
  @Override
  public void execute() {
    if (!reversed) {
      leftClimber.runVelocity(velocity);
      rightClimber.runVelocity(velocity);
    } else {
      leftClimber.runVelocity(-velocity);
      rightClimber.runVelocity(-velocity);
    }
  }

  /** Called once the command ends or is interrupted */
  @Override
  public void end(boolean interrupted) {
    leftClimber.stop();
    rightClimber.stop();

    leftClimber.softLimitEnabled(false);
    rightClimber.softLimitEnabled(false);
  }

  /** Returns true when the command should end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
