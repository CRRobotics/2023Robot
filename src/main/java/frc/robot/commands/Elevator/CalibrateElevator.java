package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class CalibrateElevator extends Command implements Constants.Elevator{

    private Elevator elevator;
    private boolean done;
    private boolean startingAtBottom;
    public CalibrateElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        done = false;
    }

    @Override
    public void initialize() {
        startingAtBottom = elevator.getBottomSwitch();
    }

    @Override
    public void execute() {
        if (startingAtBottom) {
            if (!elevator.getBottomSwitch()) {
                startingAtBottom = false;
            } else {
                elevator.setElevatorVelocity(elevatorCalibrationSpeed);
            }
            
        } else {
            if (!elevator.getBottomSwitch()) {
                elevator.setElevatorVelocity(-elevatorCalibrationSpeed);
            } else {
                elevator.setElevatorVelocity(0);
                elevator.resetElevatorEncoder();
                done = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
