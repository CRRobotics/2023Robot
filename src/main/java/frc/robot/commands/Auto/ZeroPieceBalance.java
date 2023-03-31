package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class ZeroPieceBalance extends SequentialCommandGroup {
    public ZeroPieceBalance(DriveTrain driveTrain) {
        addCommands(
            driveTrain.followTrajectoryCommand(
                PathPlanner.loadPath("1PieceBalance", Constants.Auto.constraints),
                true
            )
            // new Balance(driveTrain)
        );
    }
}
