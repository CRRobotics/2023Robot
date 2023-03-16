package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class AcquireDoubleSub extends SequentialCommandGroup{
    public AcquireDoubleSub(Elevator elevator){
        addCommands(
            new SetArmPosition(elevator, 0.292, Constants.Elevator.elbowOffset, Constants.Elevator.wristOffset).withTimeout(5),
            new SetArmPosition(elevator, 0.292, -100, 0).withTimeout(5)
        );
    }
}
