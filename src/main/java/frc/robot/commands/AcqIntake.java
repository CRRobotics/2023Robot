package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 /**
  * Runs the acquisition inward
  */
public class AcqIntake extends CommandBase
{
    Acquisition acquisition;
    public AcqIntake(Acquisition acquisition) 
    {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize()
    {
        System.out.println("running the intake");
    }
    
    @Override 
    public void execute()
    {
        double speed = 0.25;
        acquisition.spinAcq(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        acquisition.spinAcq(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}