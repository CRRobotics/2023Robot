package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.placeTopCone;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class PlaceTopDriveBackwards extends SequentialCommandGroup {
    public PlaceTopDriveBackwards(Elevator elevator, Grabber grabber, DriveTrain driveTrain) {
        addCommands(
            new placeTopCone(elevator, grabber),
            driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(1, 1),
            new PathPoint(driveTrain.getPose().getTranslation(), driveTrain.getPose().getRotation()),
            new PathPoint(driveTrain.getPose().getTranslation().plus(new Translation2d(-1, 0)), driveTrain.getPose().getRotation())),
            false)
        );
    }
}