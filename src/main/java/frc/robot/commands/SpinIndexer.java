package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.JeVoisInterface;
// import frc.robot.misc.JeVoisInterface;
import frc.robot.subsystems.Indexer;

public class SpinIndexer extends CommandBase {
    Indexer indexer;
    double angle;
    PIDController indexerPID;
    JeVoisInterface jeVoisInterface;
    public SpinIndexer(Indexer indexer, double angle) {
        this.indexer = indexer;
        addRequirements(indexer); 
        indexerPID = new PIDController(0.08, 0.02, 0);
        indexerPID.setTolerance(Math.PI / 24);
        indexerPID.enableContinuousInput(-Math.PI, Math.PI);
        jeVoisInterface = new JeVoisInterface();
    }

    @Override
    public void execute() {
        angle = NetworkTableInstance.getDefault().getTable("Visions").getEntry("ctheta").getDouble(6.0);
        SmartDashboard.putNumber("cone Angle", angle);
        SmartDashboard.putNumber("error", indexerPID.getPositionError());
        if (angle <= 10) indexer.setIndexerVelocity(indexerPID.calculate(angle, Math.PI / 2));
        else indexer.setIndexerVelocity(0);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
