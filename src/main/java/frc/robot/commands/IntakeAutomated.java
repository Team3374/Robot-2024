package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeAutomated extends Command {
  private Intake intake;

  private double velocityRPM;

  /** Creates a new IndexerAutomated */
  public IntakeAutomated(Intake intake, double velocityRPM) {
    this.intake = intake;
    addRequirements(intake);

    this.velocityRPM = velocityRPM;
  }

  /** Called when the command is initially scheduled */
  @Override
  public void initialize() {
    intake.stop();
  }

  /** Called every time the scheduler runs while the command is scheduled */
  @Override
  public void execute() {
    intake.runVelocity(-velocityRPM);
    // if (intake.getBeamBrake()) {
    //   intake.runVelocity(-velocityRPM);
    // } else {
    //   intake.stop();
    // }
  }

  /** Called once the command ends or is interrupted */
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  /** Returns true when the command should end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
