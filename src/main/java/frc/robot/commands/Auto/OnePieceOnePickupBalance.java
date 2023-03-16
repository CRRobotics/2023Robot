package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.GroundPickup;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePieceOnePickupBalance extends SequentialCommandGroup {
    public OnePieceOnePickupBalance(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new PlaceTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("2PieceA", Constants.Auto.constraints),
                false
            ),
            new GroundPickup(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1Piece1PickupBalance", Constants.Auto.constraints),
                false
            ),
            new Balance(driveTrain)
        );
    }
}
