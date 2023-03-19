package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Elevator.AutoTop;
import frc.robot.commands.grabber.Grab;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePiece2 extends SequentialCommandGroup {
    public OnePiece2(DriveTrain driveTrain, Grabber grabber, Elevator elevator) {
        addCommands(
            new Grab(grabber).withTimeout(0.5),
            new AutoTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1Piece Copy", Constants.Auto.constraints),
                false)
        );
    }
}
