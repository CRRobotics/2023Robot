package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class DriveToScoring extends CommandBase {
    DriveTrain driveTrain;
    public DriveToScoring(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Translation2d translationDifference = new Translation2d(14.66, 3.85).minus(driveTrain.getPose().getTranslation());
        Rotation2d translationRotation = new Rotation2d(translationDifference.getX(), translationDifference.getY());
        Command driveCommand = driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(1, 1),
            new PathPoint(driveTrain.getPose().getTranslation(), translationRotation, driveTrain.getPose().getRotation()),
            new PathPoint(new Translation2d(14.66, 3.85), translationRotation, Rotation2d.fromDegrees(0))),
            false);
        driveCommand.schedule();
    }
}
