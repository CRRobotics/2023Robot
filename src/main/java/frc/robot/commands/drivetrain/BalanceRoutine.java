package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;

public class BalanceRoutine extends SequentialCommandGroup {
    public BalanceRoutine(DriveTrain driveTrain, LED led) {
        addCommands(
            driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
            Constants.Auto.constraints,
            new PathPoint(driveTrain.getPose().getTranslation(), driveTrain.getPose().getRotation()),
            new PathPoint(driveTrain.getPose().getTranslation(), Rotation2d.fromDegrees(0))),
            false),
            new Balance(driveTrain, led)
        );
    }
}
