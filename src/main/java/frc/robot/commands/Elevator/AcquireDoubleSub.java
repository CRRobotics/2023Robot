package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class AcquireDoubleSub extends SequentialCommandGroup{
    public AcquireDoubleSub(Elevator elevator){
        addCommands(
            new SetArmPosition(elevator, 0.292, -162.999890, -7.498458).withTimeout(5),
            new SetArmPosition(elevator, 0.292, -100, 0).withTimeout(5)
        );
    }
}
