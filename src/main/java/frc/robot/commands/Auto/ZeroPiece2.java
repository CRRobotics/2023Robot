package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class ZeroPiece2 extends SequentialCommandGroup {
    public ZeroPiece2(DriveTrain driveTrain) {
        addCommands(
            driveTrain.followTrajectoryCommand(PathPlanner.loadPath(
                "0Piece Copy", Constants.Auto.constraints, false),
                false
            )
        );
    }
}
