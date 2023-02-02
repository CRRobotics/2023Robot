package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Constants
{
    private final CANSparkMax linearMotor = new CANSparkMax(Constants.Elevator.linearMotorID, null); //replace null

    public Elevator() {

    }


}