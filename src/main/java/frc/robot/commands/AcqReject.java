package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 /**
  * Runs the acquisition backwards based on a trigger axis
  */
public class AcqReject extends CommandBase implements Constants.Acquisition {
    Acquisition acquisition;

    public AcqReject(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void execute()
    {
        acquisition.moveAcq(downPosition);
        acquisition.spinAcq(rejectSpeed);
    }

    @Override
    public void end(boolean interrupted)
    {
        
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}