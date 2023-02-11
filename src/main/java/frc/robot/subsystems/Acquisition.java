package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Acquisition extends SubsystemBase
{
    private VictorSPX victorSPX1;
    private VictorSPX victorSPX2;
    public Acquisition()
    {
        victorSPX1 = new VictorSPX(Constants.Acquisition.acquisitionMotor1);
        victorSPX2 = new VictorSPX(Constants.Acquisition.acquisitionMotor2);
    }
    public void moveAcq (double status)
    {
        victorSPX1.set(status);
    }
    public void spinAcq (double speed)
    {
        victorSPX2.set(speed);
    }
}
