import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.upi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;

public class Elevator extends SubsystemBase{

    private final PIDController elevatorPID;
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    public Elevator(int motorID){
        elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);
        elevatorEncoder = elevatorMotor.getEncoder();
    }

    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
    }

    public void setMotorPosition(double targetPosition)
    {
        elevatorMotor.set(elevatorPID.calculate(elevatorEncoder.getPosition() * Constants.Elevator.elevatorEncoderRate, targetPosition));
    }
}