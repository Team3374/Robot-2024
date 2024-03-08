package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class RunSingleClimber extends Command {
  private Climber leftClimber;
  private Climber rightClimber;

  private double maxVelocity;

  private DoubleSupplier leftInput;
  private DoubleSupplier rightInput;

  /** Creates a new ClimberToPosition */
  public RunSingleClimber(
      Climber leftClimber,
      Climber rightClimber,
      double maxVelocity,
      DoubleSupplier leftInput,
      DoubleSupplier rightInput) {

    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;

    this.maxVelocity = maxVelocity;

    this.leftInput = leftInput;
    this.rightInput = rightInput;

    addRequirements(leftClimber, rightClimber);
  }

  /** Called when the command is initially scheduled */
  @Override
  public void initialize() {
    // leftClimber.softLimitEnabled(false);
    // rightClimber.softLimitEnabled(false);
  }

  /** Called every time the scheduler runs while the command is scheduled */
  @Override
  public void execute() {
    double leftInputAsDouble = leftInput.getAsDouble();
    double rightInputAsDouble = rightInput.getAsDouble();

    if (Math.abs(leftInputAsDouble) > 0.05) {
      leftClimber.runVelocity(leftInputAsDouble * maxVelocity);
    } else {
      leftClimber.stop();
    }

    if (Math.abs(rightInputAsDouble) > 0.05) {
      rightClimber.runVelocity(rightInputAsDouble * maxVelocity);
    } else {
      rightClimber.stop();
    }

    SmartDashboard.putString("Climber Mode", "Manual");

    SmartDashboard.putNumber("Left Input", leftInputAsDouble);
    SmartDashboard.putNumber("Right Input", rightInputAsDouble);
  }

  /** Called once the command ends or is interrupted */
  @Override
  public void end(boolean interrupted) {
    leftClimber.stop();
    rightClimber.stop();

    SmartDashboard.putString("Climber Mode", "Not Manual");

    // leftClimber.softLimitEnabled(false);
    // rightClimber.softLimitEnabled(false);
  }

  /** Returns true when the command should end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
