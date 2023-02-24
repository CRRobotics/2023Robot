package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetArmPosition extends CommandBase{
    private Elevator elevator;
    private double x;
    private double y;
    private double elevatorRotations;
    private double pos = 0.1;


    public SetArmPosition(Elevator elevator, double x, double y, double elevatorRotations) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevatorRotations = elevatorRotations;
        this.x = x;
        this.y = y;
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setArmMotors(x, y);
        elevator.setElevatorPosition(pos);
        pos += 0.2;
    }
    
}