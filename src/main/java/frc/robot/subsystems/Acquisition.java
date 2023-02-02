package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.ControllerWrapper;
public class Acquisition extends SubsystemBase
{
    private WPI_VictorSPX victorSPX1 = new WPI_VictorSPX(Constants.Acquisition.acquisitionMotor1);
    private WPI_VictorSPX victorSPX2 = new WPI_VictorSPX(Constants.Acquisition.acquisitionMotor2);
    public Acquisition(){
        if(ControllerWrapper.l2.getLeftTriggerAxis() > 0.5)
        {
            victorSPX1.set(1);
            victorSPX2.set(1);
        }
    }
}
