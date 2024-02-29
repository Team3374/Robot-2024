package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class RunClimber extends Command {
  private Climber leftClimber;
  private Climber rightClimber;

  private double maxVelocity;
  private double limit;

  private DoubleSupplier input;

  /** Creates a new ClimberToPosition */
  public RunClimber(
      Climber leftClimber,
      Climber rightClimber,
      double maxVelocity,
      double limit,
      DoubleSupplier input) {
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;

    this.maxVelocity = maxVelocity;
    this.limit = limit;

    this.input = input;

    addRequirements(leftClimber, rightClimber);
  }

  /** Called when the command is initially scheduled */
  @Override
  public void initialize() {
    leftClimber.stop();
    rightClimber.stop();
  }

  /** Called every time the scheduler runs while the command is scheduled */
  @Override
  public void execute() {
    double currentInput = input.getAsDouble();

    if (currentInput > 0.05 && leftClimber.getPosition() < limit) {
      leftClimber.runVelocity(maxVelocity * currentInput);
    } else if (currentInput < 0.05 && leftClimber.getPosition() > limit) {
      leftClimber.runVelocity(maxVelocity * currentInput);
    } else {
      leftClimber.stop();
    }

    if (currentInput > 0.05 && rightClimber.getPosition() < limit) {
      rightClimber.runVelocity(maxVelocity * currentInput);
    } else if (currentInput < 0.05 && rightClimber.getPosition() > limit) {
      rightClimber.runVelocity(maxVelocity * currentInput);
    } else {
      rightClimber.stop();
    }
  }

  /** Called once the command ends or is interrupted */
  @Override
  public void end(boolean interrupted) {
    leftClimber.stop();
    rightClimber.stop();
  }

  /** Returns true when the command should end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
