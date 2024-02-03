package frc.robot.misc;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface ControllerWrapper {
    XboxController driveController = new XboxController(0);
    XboxController controlController = new XboxController(1);

    public static JoystickButton controllerA = new JoystickButton(controlController, 0);
    public static JoystickButton controllerB = new JoystickButton(controlController, 1);
    public static JoystickButton controllerX = new JoystickButton(controlController, 2);
    public static JoystickButton controllerY = new JoystickButton(controlController,3);

    public static JoystickButton controllerLeftTopTrigger = new JoystickButton(controlController,4);
    public static JoystickButton controllerRightTopTrigger = new JoystickButton(controlController,5);

    public static JoystickButton controllerTopLeftWeirdButton = new JoystickButton(controlController,6);
    public static JoystickButton controllerTopRightWeirdButton = new JoystickButton(controlController,7);

    public static JoystickButton controllerLeftJoystickIn = new JoystickButton(controlController, 8);
    public static JoystickButton controllerRightJoystickIn = new JoystickButton(controlController, 9);

    public static JoystickButton driverA = new JoystickButton(driveController, 0);
    public static JoystickButton driverB = new JoystickButton(driveController, 1);
    public static JoystickButton driverX = new JoystickButton(driveController, 2);
    public static JoystickButton driverY = new JoystickButton(driveController,3);

    public static JoystickButton driverLeftTopTrigger = new JoystickButton(driveController,4);
    public static JoystickButton driverRightTopTrigger = new JoystickButton(driveController,5);

    public static JoystickButton driverTopLeftWeirdButton = new JoystickButton(driveController,6);
    public static JoystickButton driverTopRightWeirdButton = new JoystickButton(driveController,7);

    public static JoystickButton driverLeftJoystickIn = new JoystickButton(driveController, 8);
    public static JoystickButton driverRightJoystickIn = new JoystickButton(driveController, 9);
}
