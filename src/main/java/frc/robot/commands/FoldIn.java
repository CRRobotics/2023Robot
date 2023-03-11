package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.SetArmPosition;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class FoldIn extends SequentialCommandGroup{
    public FoldIn(Elevator elevator){
        addCommands(
            new SetArmPosition(elevator, 0.33, -90, 0).withTimeout(5),
            new SetArmPosition(elevator, 0.33, -162.999890, -7.498458).withTimeout(5)
        );
    }
}
