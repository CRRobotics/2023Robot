package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Elevator.GroundPickupCube;
import frc.robot.commands.Elevator.AutoTop;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePieceOnePickupBalance extends SequentialCommandGroup {
    public OnePieceOnePickupBalance(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new InstantCommand(() -> {Robot.setPieceType(PieceType.Cube);}),
            new AutoTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("2PieceA", Constants.Auto.constraints),
                true
            ),
            new InstantCommand(() -> {Robot.setPieceType(PieceType.Cone);}),
            new GroundPickupCube(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1Piece1PickupBalance", Constants.Auto.constraints),
                true
            )
            // new Balance(driveTrain)
        );
    }
}
