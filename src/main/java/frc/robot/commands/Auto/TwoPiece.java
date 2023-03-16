package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Elevator.GroundPickup;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new InstantCommand(() -> {Robot.setPieceType(PieceType.Cube);}),
            new PlaceTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("2PieceA", Constants.Auto.constraints),
                true
            ),
            new InstantCommand(() -> {Robot.setPieceType(PieceType.Cone);}),
            new GroundPickup(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("2PieceB", Constants.Auto.constraints),
                true
            ),
            new PlaceTop(elevator, grabber)
        );
    }
}
