package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class GroundPickup extends SequentialCommandGroup {
    public GroundPickup(Elevator elevator, Grabber grabber) {
        addCommands(
            new SetArmPosition(elevator, 0, 0, 0).withTimeout(5),
            new SetArmPosition(elevator, 0, 0, 0).withTimeout(5)
        );
    }
}