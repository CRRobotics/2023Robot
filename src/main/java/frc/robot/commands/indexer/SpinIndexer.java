package frc.robot.commands.indexer;

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

    /**
     * Creates a <code>SpinIndexer</code> with a given indexer
     * @param indexer The indexer to be referenced by the <code>SpinIndexer</code>
     */
    public SpinIndexer(Indexer indexer) {
        this.indexer = indexer;
        this.angle = NetworkTableInstance.getDefault().getTable("Visions").getEntry("ctheta").getDouble(6.0);
        addRequirements(indexer);
        indexerPID = new PIDController(0.08, 0.02, 0);
        indexerPID.setTolerance(Math.PI / 24);
        indexerPID.enableContinuousInput(-Math.PI, Math.PI);
        jeVoisInterface = new JeVoisInterface();
    }

    /**
     * Finds the angle of the object relative to the robot and spins the plate to put the object in the correct position.
     */
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
