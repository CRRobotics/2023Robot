package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveModule;

public class JoystickDrive extends CommandBase{
    DriveTrain driveTrain;
    XboxController controller = new XboxController(0);

    public JoystickDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // double xSpeed = MathUtil.applyDeadband(controller.getRawAxis(1), 0.06);
        // double ySpeed = MathUtil.applyDeadband(controller.getRawAxis(0), 0.06);
        // double rot = MathUtil.applyDeadband(controller.getRawAxis(2), 0.06);
        double xSpeed = MathUtil.applyDeadband(controller.getLeftY(), 0.06);
        double ySpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.06);
        double rot = MathUtil.applyDeadband(controller.getRightX(), 0.06);
        boolean fieldRelative = true;
        xSpeed *= Constants.Drive.maxSpeed;
        ySpeed *= Constants.Drive.maxSpeed;
        rot *= Constants.Drive.maxAngularSpeed;

        SwerveModuleState[] swerveModuleStates = Constants.Drive.driveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(driveTrain.getHeading()))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        driveTrain.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
