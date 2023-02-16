package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.JeVoisInterface;
import frc.robot.subsystems.Indexer;

public class SpinIndexer extends CommandBase {
    Indexer indexer;
    double angle;
    PIDController indexerPID;
    JeVoisInterface jeVoisInterface;
    public SpinIndexer(Indexer indexer, double angle) {
        this.indexer = indexer;
        addRequirements(indexer); 
        indexerPID = new PIDController(0.01, 0, 0);
        jeVoisInterface = new JeVoisInterface();
    }

    @Override
    public void execute() {
        indexer.setIndexerVelocity(indexerPID.calculate(jeVoisInterface.getConeAngle(), angle));
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
