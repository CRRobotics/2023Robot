package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetArmPosition extends CommandBase{
    private Elevator elevator;
    private double x;
    private double y;
    private double elevatorRotations;
    XboxController controller = new XboxController(0);


    public SetArmPosition(Elevator elevator, double x, double y, double elevatorRotations) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevatorRotations = elevatorRotations;
        this.x = x;
        this.y = y;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setArmMotors(controller.getLeftY(), 0);
        // elevator.setElevatorPosition(elevatorRotations);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopArmMotors();
    }
}