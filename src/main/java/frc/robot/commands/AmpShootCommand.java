package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

public class AmpShootCommand extends Command {
  private Flywheel flywheel;

  private DoubleSupplier rightTrigger;

  public AmpShootCommand(Flywheel flywheel, DoubleSupplier rightTrigger) {
    this.flywheel = flywheel;
    this.rightTrigger = rightTrigger;

    addRequirements(flywheel);
  }

  /** Called when the command is initially scheduled */
  @Override
  public void initialize() {
    flywheel.stop();
  }

  /** Called every time the scheduler runs while the command is scheduled */
  @Override
  public void execute() {
    double velocityRPM = rightTrigger.getAsDouble();
    flywheel.runVelocity(velocityRPM * 50, velocityRPM * 2250);
  }

  /** Called once the command ends or is interrupted */
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  /** Returns true when the command should end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
