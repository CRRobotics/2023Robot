package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class SpinIndexer extends CommandBase {
    Indexer indexer;
    double angle;
    public SpinIndexer(Indexer indexer, double angle) {
        this.indexer = indexer;
        addRequirements(indexer);
        
    }

    @Override
    public void execute() {
        indexer.setIndexerVelocity(1);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
