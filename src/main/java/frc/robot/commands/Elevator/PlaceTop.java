package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class PlaceTop extends SequentialCommandGroup{
    public PlaceTop(Elevator elevator, Grabber grabber){
        addCommands(
            new Grab(grabber).withTimeout(0.5),
            new SetArmPosition(elevator, 0.34, Constants.Elevator.elbowOffset, Constants.Elevator.wristOffset).withTimeout(5),
            new SetArmPosition(elevator, 0.34, -75, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.755, -75, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.755, -10, 0).withTimeout(5),
            new Ungrab(grabber).withTimeout(1.5),
            new SetArmPosition(elevator, 0.755, -75, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.34, -75, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.1, Constants.Elevator.elbowSafe, Constants.Elevator.wristSafe).withTimeout(5)
        );
    }
}
