package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class ZeroPiece extends SequentialCommandGroup {
    public ZeroPiece(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            driveTrain.followTrajectoryCommand(PathPlanner.loadPath(
                "0Piece", Constants.Auto.constraints),
                true
            )
        );
    }
}
