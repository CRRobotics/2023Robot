package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class PlaceTop extends SequentialCommandGroup{
    public PlaceTop(Elevator elevator, Grabber grabber){
        addCommands(
            new SetArmPosition(elevator, 0.34, -163, -7.5).withTimeout(5),
            new SetArmPosition(elevator, 0.34, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.76, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.76, -10, 0).withTimeout(5),
            new Ungrab(grabber).withTimeout(0.3),
            new Grab(grabber).withTimeout(0.3),
            new SetArmPosition(elevator, 0.76, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.34, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.1, -157, -6).withTimeout(5)
        );
    }
}