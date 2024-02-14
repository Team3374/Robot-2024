package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerAutomated extends Command {
    private Indexer indexer;

    private double velocityRPM;

    /** Creates a new IndexerAutomated */
    public IndexerAutomated(Indexer indexer, Double velocityRPM) {
        this.indexer = indexer;
        addRequirements(indexer);

        this.velocityRPM = velocityRPM;
    }

    /** Called when the command is initially scheduled */
    @Override
    public void initialize() {
        indexer.stop();
    }

    /** Called every time the scheduler runs while the command is scheduled */
    @Override
    public void execute() {
        if (!indexer.getBeamBrake()) {
        indexer.runVelocity(velocityRPM);
        } else {
        indexer.stop();
        }

    }

    /** Called once the command ends or is interrupted */
    @Override
    public void end(boolean interrupted) {
    }

    /** Returns true when the command should end */
    @Override
    public boolean isFinished() {
        return false;
    }
}
