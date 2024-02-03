package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class AcquireDoubleSub extends SequentialCommandGroup{
    public AcquireDoubleSub(Elevator elevator){
            addCommands(
                new SetArmPosition(elevator, 0.2, 0, Constants.Elevator.elbowSafe, 0, Constants.Elevator.wristSafe, 0).withTimeout(5),
                // new SetArmPosition(elevator, 0.2, Constants.Elevator.elbowSafe, Constants.Elevator.wristSafe).withTimeout(5),
                new SetArmPosition(elevator, 0.2, -93.5, 26).withTimeout(5)
            );
        
    }
}
