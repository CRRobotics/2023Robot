package frc.robot.misc;

import edu.wpi.first.wpilibj.XboxController;

public interface ControllerWrapper {
    XboxController driveController = new XboxController(0);
    XboxController controlController = new XboxController(1);
    XboxController l2 = new XboxController(3);
    // port incorrect for l2
}
