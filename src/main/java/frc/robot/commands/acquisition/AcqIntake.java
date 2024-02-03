package frc.robot.commands.acquisition;

import frc.robot.misc.Constants;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;

 /**
  * Runs the acquisition inward
  */
public class AcqIntake extends Command implements Constants.Acquisition {
    Acquisition acquisition;

    public AcqIntake(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }
    
    @Override 
    public void execute() {
        acquisition.spinAcq(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.spinAcq(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}