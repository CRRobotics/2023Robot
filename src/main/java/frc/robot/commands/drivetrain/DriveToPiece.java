package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;
import frc.robot.subsystems.DriveTrain;

public class DriveToPiece extends CommandBase {
    DriveTrain driveTrain;
    PIDController pid;
    public DriveToPiece(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        pid = new PIDController(SmartDashboard.getNumber("piece auto P", 0), 0, 0);
    }

    @Override
    public void execute() {
    }

}
