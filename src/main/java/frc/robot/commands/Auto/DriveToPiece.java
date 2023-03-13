package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.NetworkTableWrapper;
import frc.robot.subsystems.DriveTrain;

public class DriveToPiece extends CommandBase {
    DriveTrain driveTrain;
    public DriveToPiece(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        double[] pieceData;
        if (NetworkTableWrapper.getArray("RealSense", "Cone")[0] == 1) {
            pieceData = NetworkTableWrapper.getArray("RealSense", "Cone");
        } else {
            pieceData = NetworkTableWrapper.getArray("RealSense", "Cube");
        }
        Translation2d pieceDifference = new Translation2d(pieceData[1], pieceData[3]);
        Rotation2d translationRotation = new Rotation2d(pieceDifference.getX(), pieceDifference.getY());
        Command driveCommand = driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(1, 1),
            new PathPoint(driveTrain.getPose().getTranslation(), translationRotation, driveTrain.getPose().getRotation()),
            new PathPoint(driveTrain.getPose().getTranslation().plus(pieceDifference), translationRotation, Rotation2d.fromDegrees(0))),
            false);
        driveCommand.schedule();
    }

}
