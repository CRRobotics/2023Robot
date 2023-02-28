package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class Place extends CommandBase
{
    Elevator elevator;
    int level; //1 for first level, 2 for middle level, 3 for top level  

    public Place(Elevator elevator, int level)
    {
        this.elevator = elevator;
        this.level = level;
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