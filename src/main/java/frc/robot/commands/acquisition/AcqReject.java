package frc.robot.commands.acquisition;

import frc.robot.misc.Constants;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;

 /**
  * Runs the acquisition backwards based on a trigger axis
  */
public class AcqReject extends Command implements Constants.Acquisition {
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