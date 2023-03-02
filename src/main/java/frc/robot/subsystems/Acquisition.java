package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Acquisition extends SubsystemBase implements Constants.Acquisition {
    // private VictorSPX lowerMotor;
    private VictorSPX highMotor;
    private VictorSPX lowMotor;
    private boolean lowered;

    public Acquisition() {
        highMotor = new VictorSPX(highMotorID);
        lowMotor = new VictorSPX(lowMotorID);
        lowMotor.follow(highMotor);
        lowered = false;
    }

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

    /**
     * Moves acquisition
     * @param elevate The distance to elevate
     */
    public void moveAcq (double elevate)
    {
        // lowMotor.set(ControlMode.Position, acqPID.calculate(lowerMotor.getClosedLoopError(), elevate));
    }

     /**
     * Spins the acquisition
     * @param speed The speed of the acquisition's rotation
     */
    public void spinAcq(double speed)
    {
        highMotor.set(ControlMode.PercentOutput, speed);
    }
}
