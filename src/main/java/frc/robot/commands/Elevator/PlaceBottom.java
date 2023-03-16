package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class PlaceBottom extends SequentialCommandGroup {
    public PlaceBottom(Elevator elevator, Grabber grabber){
        addCommands(
            new SetArmPosition(elevator, 0.1, Constants.Elevator.elbowOffset, Constants.Elevator.wristOffset).withTimeout(5),
            new SetArmPosition(elevator, 0.1, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.1, -90, -40).withTimeout(5),
            new Ungrab(grabber).withTimeout(0.3),
            new Grab(grabber).withTimeout(0.3),
            new SetArmPosition(elevator, 0.1, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.1, -157, -6).withTimeout(5)
        );
    }
}
