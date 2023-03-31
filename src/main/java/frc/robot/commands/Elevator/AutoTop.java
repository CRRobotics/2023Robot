package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class AutoTop extends SequentialCommandGroup{
    public AutoTop(Elevator elevator, Grabber grabber) {
        addCommands(
            new Grab(grabber).withTimeout(.7),
            new PlaceTop(elevator),
            new Ungrab(grabber).withTimeout(.7),
            new FoldIn(elevator)

        );
    }
}
