package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class CalibrateElevator extends CommandBase implements Constants.Elevator{

    private Elevator elevator;
    private boolean done;
    public CalibrateElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        done = false;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!elevator.getBottomSwitch()) {
            elevator.setElevatorVelocity(elevatorCalibrationSpeed);
        } else {
            elevator.setElevatorVelocity(0);
            elevator.resetElevatorEncoder();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
