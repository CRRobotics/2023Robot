package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
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
    double startTime;


    public SetArmPosition(Elevator elevator, double x, double y, double elevatorRotations) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevatorRotations = elevatorRotations;
        this.x = x;
        this.y = y;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        elevator.setPosition();
    }

    @Override
    public void execute() {
        elevator.setArmMotors(0, 0);
        elevator.setElevatorPosition(controller.getLeftY(), Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopArmMotors();
    }
}