package frc.robot.misc;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

public interface ControllerWrapper {
    XboxController driveController = new XboxController(0);
    XboxController controlController = new XboxController(1);

   public static DPadButton DriverDPadUp = new DPadButton(DriverController, 0);
    public static DPadButton DriverDPadRight = new DPadButton(DriverController, 90);
    public static DPadButton DriverDPadDown = new DPadButton(DriverController, 180);
    public static DPadButton DriverDPadLeft = new DPadButton(DriverController, 270);

    public static JoystickButton DriverButtonY = new JoystickButton(DriverController, XboxController.Button.kY.value);
    public static JoystickButton DriverButtonX = new JoystickButton(DriverController, XboxController.Button.kX.value);
    public static JoystickButton DriverButtonA = new JoystickButton(DriverController, XboxController.Button.kA.value);
    public static JoystickButton DriverButtonB = new JoystickButton(DriverController, XboxController.Button.kB.value);

    public static JoystickButton DriverRightBumper = new JoystickButton(DriverController, XboxController.Button.kRightBumper.value);
    public static JoystickButton DriverLeftBumper = new JoystickButton(DriverController, XboxController.Button.kLeftBumper.value);

    //Controller XBOX Controller
    public static DPadButton ControlDPadUp = new DPadButton(ControlController, 0);
    public static DPadButton ControlDPadRight = new DPadButton(ControlController, 90);
    public static DPadButton ControlDPadDown = new DPadButton(ControlController, 180);
    public static DPadButton ControlDPadLeft = new DPadButton(ControlController, 270);

    public static JoystickButton ControlButtonY = new JoystickButton(ControlController, XboxController.Button.kY.value);
    public static JoystickButton ControlButtonX = new JoystickButton(ControlController, XboxController.Button.kX.value);
    public static JoystickButton ControlButtonA = new JoystickButton(ControlController, XboxController.Button.kA.value);
    public static JoystickButton ControlButtonB = new JoystickButton(ControlController, XboxController.Button.kB.value);

    public static JoystickButton ControlRightBumper = new JoystickButton(ControlController, XboxController.Button.kRightBumper.value);
    public static JoystickButton ControlLeftBumper = new JoystickButton(ControlController, XboxController.Button.kLeftBumper.value);
    
}
