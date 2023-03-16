package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveToScoring extends CommandBase implements Constants.Auto{
    DriveTrain driveTrain;
    public DriveToScoring(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = new Pose2d(new Translation2d(14.66, 3.85), Rotation2d.fromDegrees(0)); // top node on red
        Translation2d robotPosition = driveTrain.getPose().getTranslation(); // current position
        Translation2d translationDifference = targetPose.getTranslation().minus(robotPosition); // difference between target and current
        // calculates wheel angle needed to target from x and y components
        Rotation2d translationRotation = new Rotation2d(translationDifference.getX(), translationDifference.getY());
        Command driveCommand = driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(maxSpeed, maxAcceleration),
            new PathPoint(robotPosition, translationRotation, driveTrain.getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, targetPose.getRotation())), // ending pose
            false);
        driveCommand.schedule();
    }
}
