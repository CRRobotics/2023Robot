package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;

public class Balance extends CommandBase implements Constants.Auto {
    DriveTrain driveTrain;
    PIDController pidController;
    LED led;

    public Balance(DriveTrain driveTrain, LED led) {
        this.driveTrain = driveTrain;
        this.led = led;
        addRequirements(driveTrain, led);
    }

    @Override
    public void initialize() {
        pidController = new PIDController(balanceP, balanceI, balanceD);
        pidController.setTolerance(balanceTolerance);
        led.showAuto();
    }

    @Override
    public void execute() {
        pidController.setPID(SmartDashboard.getNumber("drivetrain/balanceP", 0),
        SmartDashboard.getNumber("drivetrain/balanceI", 0),
        SmartDashboard.getNumber("drivetrain/balanceD", 0));
        driveTrain.setMotorSpeeds(pidController.calculate(driveTrain.getPitch()));
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setMotorSpeeds(0);
    }


}
