package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Acquisition extends SubsystemBase
{
    // private VictorSPX lowerMotor;
    private VictorSPX spinMotor;
    private ControlMode mode;
    private PIDController acqPID;
    private boolean lowered;

    /**
     *Returns whether the acquisition is lowered
     *@return True if the acquisition is lowered, false otherwise
     */
    public boolean isLowered() {
        return lowered;
    }

    /** 
     * Sets lowered to input boolean
     */
    public void setLowered(boolean lowered) {
        this.lowered = lowered;
    }

    public Acquisition()
    {
        mode = ControlMode.Velocity;
        acqPID = new PIDController(0.001, 0, 0);
        // lowerMotor = new VictorSPX(Constants.Acquisition.acquisitionMotor1);
        spinMotor = new VictorSPX(Constants.Acquisition.acquisitionMotor2);
        lowered = false;
    }

    /**
     * Moves acquisition
     * @param elevate The distance to elevate
     */
    public void moveAcq (double elevate)
    {
        // lowerMotor.set(ControlMode.Position, acqPID.calculate(lowerMotor.getClosedLoopError(), elevate));
    }

     /**
     * Spins the acquisition
     * @param speed The speed of the acquisition's rotation
     */
    public void spinAcq(double speed)
    {
        spinMotor.set(ControlMode.PercentOutput, speed);
    }
}
