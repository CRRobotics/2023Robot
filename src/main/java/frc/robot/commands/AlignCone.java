package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
