package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveToScoring extends CommandBase implements Constants.Auto{
    DriveTrain driveTrain;
    double[] scoringPositions = {
        0.46, 1.07, 1.64, 2.2, 2.74, 1.81, 3.87, 4.42, 5.07 // y positions in m of 9 scoring positions
    };

    public DriveToScoring(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Translation2d robotPosition = driveTrain.getPose().getTranslation(); // current position

        // calculates which position is closest
        double[] distances = new double[9];
        int minDistanceIndex = 0;
        for (int i = 0; i < distances.length; i++) {
            distances[i] = robotPosition.getDistance(new Translation2d(1.78, scoringPositions[i]));
            if (distances[i] < distances[minDistanceIndex]) {
                minDistanceIndex = i;
            }
        }

        Pose2d targetPose = new Pose2d(new Translation2d(1.78, scoringPositions[minDistanceIndex]), Rotation2d.fromDegrees(0)); // top node on red
        Translation2d translationDifference = targetPose.getTranslation().minus(robotPosition); // difference between target and current
        // calculates wheel angle needed to target from x and y components
        Rotation2d translationRotation = new Rotation2d(translationDifference.getX(), translationDifference.getY());
        Command driveCommand = driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            Constants.Auto.constraints,
            new PathPoint(robotPosition, translationRotation, driveTrain.getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, targetPose.getRotation())), // ending pose
            false);
        driveCommand.schedule();
    }
}
