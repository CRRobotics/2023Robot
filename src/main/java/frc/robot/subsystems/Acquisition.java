package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Acquisition extends SubsystemBase
{
    private VictorSPX victorSPX1;
    private VictorSPX victorSPX2;
    private ControlMode mode;
    public Acquisition()
    {
        mode = ControlMode.Velocity;
        victorSPX1 = new VictorSPX(Constants.Acquisition.acquisitionMotor1);
        victorSPX2 = new VictorSPX(Constants.Acquisition.acquisitionMotor2);
    }
    public void moveAcq (double elevate)
    {
        victorSPX1.set(mode, elevate);
    }
    public void spinAcq (double speed)
    {
        victorSPX2.set(mode, speed);
    }
}
