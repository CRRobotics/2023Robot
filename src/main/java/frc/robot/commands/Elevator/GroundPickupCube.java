package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.grabber.Grab;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class GroundPickupCube extends SequentialCommandGroup {
    public GroundPickupCube(Elevator elevator, Grabber grabber) {
        addCommands( // Elevator: 0, Elbow: 52.2, Wrist: 0.1
            new SetArmPosition(elevator, 0, Constants.Elevator.elbowSafe, Constants.Elevator.wristSafe).withTimeout(5),
            new SetArmPosition(elevator, 0, 52.2, Constants.Elevator.wristSafe).withTimeout(5),
            new SetArmPosition(elevator, 0, 52.2, 5)
        );
    }
}
