package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;

public class ResetArmEncoders extends CommandBase {
    private Elevator elevator;

    public ResetArmEncoders(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.resetElevatorEncoder();
    }
    
}
