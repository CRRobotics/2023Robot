package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class OnePieceBalance extends SequentialCommandGroup {
    public OnePieceBalance(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new PlaceTop(elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1PieceBalance", Constants.Auto.constraints), 
                false),
            new Balance(driveTrain)
        );
    }
}
