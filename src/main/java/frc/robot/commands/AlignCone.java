package frc.robot.commands;
import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;

public class AlignCone extends CommandBase{
    Indexer indexer;

    public AlignCone(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        
    }

    @Override
    public void initialize() {
        indexer.turnToAngle(NetworkTableWrapper.getData(Constants.Indexer.indexerCamTable, "coneAngle")  - indexer.getAngle());
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
