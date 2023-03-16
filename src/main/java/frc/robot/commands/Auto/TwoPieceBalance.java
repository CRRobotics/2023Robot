package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class TwoPieceBalance extends SequentialCommandGroup {
    public TwoPieceBalance(DriveTrain driveTrain, Elevator elevator, Grabber grabber) {
        addCommands(
            new TwoPiece(driveTrain, elevator, grabber),
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("2PieceBalance", Constants.Auto.constraints),
                true
            ),
            new Balance(driveTrain)
        );
    }
}
