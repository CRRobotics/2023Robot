package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class FoldIn extends SequentialCommandGroup{
    public FoldIn(Elevator elevator){
        addCommands(
            new SetArmPosition(elevator, 0.33, -90, 10).withTimeout(5),
            
            // new SetArmPosition(elevator, 0.19, -Constants.Elevator.elevatorMaxVelocity, -90, 0, 10, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.1, Constants.Elevator.elbowSafe, Constants.Elevator.wristSafe).withTimeout(5)
        );
    }
}
