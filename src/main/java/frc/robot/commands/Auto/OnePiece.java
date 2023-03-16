package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePiece extends SequentialCommandGroup {
    //orewa monkey d luffy
    public OnePiece(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new InstantCommand(() -> {Robot.setPieceType(PieceType.Cube);}),
            new PlaceTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1Piece", Constants.Auto.constraints),
                true)
        );
    }
}
