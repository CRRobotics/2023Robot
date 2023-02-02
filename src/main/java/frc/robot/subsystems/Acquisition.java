package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.misc.ControllerWrapper;
public class Acquisition implements ControllerWrapper{
    private WPI_VictorSPX victorSPX1 = new WPI_VictorSPX(0);
    double aOn = l2.getLeftTriggerAxis();
}
