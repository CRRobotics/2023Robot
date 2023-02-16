import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetArmPosition extends CommandBase{
    private Elevator elevator;
    public SetArmPosition(Elevator elevator, double z, double y) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setArmMotors(0, 0);
    }
}