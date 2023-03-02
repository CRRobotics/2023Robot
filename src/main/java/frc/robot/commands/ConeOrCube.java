package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ConeOrCube extends CommandBase
{
    Elevator elevator;
    
    public ConeOrCube(Elevator elevator)
    {
        this.elevator = elevator; 
        elevator.setConeOrCube();
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        
    }
    
    @Override
    public void execute()
    {

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
