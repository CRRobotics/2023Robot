package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;
import frc.robot.subsystems.Indexer;

public class AlignCone extends CommandBase{
    Indexer indexer;

    /**
     * Creates an AlignCone object referencing the indexer given.
     * @param indexer the indexer given.
     */
    public AlignCone(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        
    }

    @Override
    public void initialize() {
        indexer.turnToAngle(NetworkTableWrapper.getDouble(Constants.Indexer.indexerCamTable, "coneAngle")  - indexer.getAngle());
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
