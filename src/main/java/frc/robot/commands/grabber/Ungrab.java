package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class Ungrab extends CommandBase{
    Grabber grabber;
    XboxController controller = new XboxController(0);
    
    public Ungrab(Grabber grabber)
    {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        grabber.setSpeed(-0.1);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
