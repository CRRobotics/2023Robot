package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePiece extends SequentialCommandGroup {
    //orewa monkey d
    public OnePiece(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new PlaceTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1Piece", Constants.Auto.constraints),
                false)
        );
    }
}
