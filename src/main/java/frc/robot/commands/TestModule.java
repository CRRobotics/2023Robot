package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TestModule extends CommandBase {
    DriveTrain driveTrain;
    XboxController controller = new XboxController(0);
    
    public TestModule(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = SmartDashboard.getNumber("speed", 0);
        double angle = SmartDashboard.getNumber("angle", 0);
        driveTrain.setModuleStates(new SwerveModuleState[]{new SwerveModuleState(speed, new Rotation2d(angle))});
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
