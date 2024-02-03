package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.Grabber;

public class Grab extends Command {
    Grabber grabber;
    XboxController controller = new XboxController(0);
    
    public Grab(Grabber grabber)
    {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute()
    {
        if (Robot.getPieceType() == PieceType.Cone) {
            grabber.setSpeed(-0.6);
        } else if (Robot.getPieceType() == PieceType.Cube) {
            grabber.setSpeed(0.6);
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabber.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
