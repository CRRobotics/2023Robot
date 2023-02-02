package frc.robot.misc;

import edu.wpi.first.wpilibj.XboxController;

public interface ControllerWrapper {
    XboxController driveController = new XboxController(0);
    XboxController controlController = new XboxController(1);
    
}
