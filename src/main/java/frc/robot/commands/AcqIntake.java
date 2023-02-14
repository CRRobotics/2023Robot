package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AcqIntake extends CommandBase
{
    Acquisition acquisition;
    XboxController controller = new XboxController(0);
    public AcqIntake(Acquisition acquisition) 
    {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize()
    {

    }
    
    @Override 
    public void execute()
    {
        acquisition.moveAcq(90);
        double speed = controller.getLeftTriggerAxis();
        acquisition.spinAcq(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        acquisition.moveAcq(-90);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}