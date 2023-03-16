package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase implements Constants.Auto {
    DriveTrain driveTrain;
    PIDController pidController;

    public Balance(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        pidController = new PIDController(balanceP, balanceI, balanceD);
        pidController.setTolerance(balanceTolerance);
    }

    @Override
    public void execute() {
        driveTrain.setMotorSpeeds(pidController.calculate(driveTrain.getPitch()));
    }


}