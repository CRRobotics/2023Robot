package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
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
        if (NetworkTableWrapper.getArray("Detector", "Cone")[0] == 1) {
            pieceData = NetworkTableWrapper.getArray("Detector", "Cone");
        } else {
            pieceData = NetworkTableWrapper.getArray("Detector", "Cube");
        }
        Translation2d pieceDifference = new Translation2d(pieceData[1], pieceData[3]);
        Translation2d currentPosition = driveTrain.getPose().getTranslation();
        Translation2d targetPosition = currentPosition.plus(pieceDifference.rotateBy(currentPosition.getAngle())); // bruh this won't work
        Rotation2d translationRotation = new Rotation2d(pieceDifference.getX(), pieceDifference.getY());
        Command driveCommand = driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            Constants.Auto.constraints,
            new PathPoint(currentPosition, translationRotation, driveTrain.getPose().getRotation()),
            new PathPoint(targetPosition, translationRotation, Rotation2d.fromDegrees(0))),
            false);
        driveCommand.schedule();
    }

}
