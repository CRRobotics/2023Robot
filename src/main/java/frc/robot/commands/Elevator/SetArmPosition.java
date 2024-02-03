package frc.robot.commands.Elevator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.misc.Constants;
import frc.robot.misc.PieceType;
import frc.robot.subsystems.Elevator;

public class SetArmPosition extends Command implements Constants.Elevator {
    private Elevator elevator;
    private double elbowPosition;
    private double elbowVelocity;
    private double wristPosition;
    private double wristVelocity;
    private double elevatorPosition;
    private double elevatorVelocity;
    XboxController controller = new XboxController(0);
    double startTime;

    private TrapezoidProfile.State elevatorSetpoint;
    private TrapezoidProfile.State elevatorGoal;
    private TrapezoidProfile elevatorProfile;

    private TrapezoidProfile.State elbowSetpoint;
    private TrapezoidProfile.State elbowGoal;
    private TrapezoidProfile elbowProfile;

    private TrapezoidProfile.State wristSetpoint;
    private TrapezoidProfile.State wristGoal;
    private TrapezoidProfile wristProfile;

    public SetArmPosition(Elevator elevator, double elevatorPosition, double elevatorVelocity, double elbowPosition, double elbowVelocity, double wristPosition, double wristVelocity) {
        this.elevator = elevator;
        this.elevatorPosition = elevatorPosition;
        this.elbowPosition = elbowPosition * (Math.PI / 180);
        this.wristPosition = wristPosition * (Math.PI / 180);
        this.elevatorVelocity = elevatorVelocity;
        this.elbowVelocity = elbowVelocity;
        this.wristVelocity = wristVelocity;
        addRequirements(elevator);
    }

    public SetArmPosition(Elevator elevator, double elevatorPosition, double elbowPosition, double wristPosition) {
    // this(elevator, elevatorPosition, 0, elbowPosition, 0, wristPosition, 0);
        this.elevator = elevator;
        this.elevatorPosition = elevatorPosition;
        this.elbowPosition = elbowPosition * (Math.PI / 180);
        this.wristPosition = wristPosition * (Math.PI / 180);
        this.elevatorVelocity = 0;
        this.elbowVelocity = 0;
        this.wristVelocity = 0;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        elevator.setPosition();

        elevatorGoal = new TrapezoidProfile.State(elevatorPosition, elevatorVelocity);
        elbowGoal = new TrapezoidProfile.State(elbowPosition, elbowVelocity);
        wristGoal = new TrapezoidProfile.State(wristPosition, wristVelocity);

        System.out.println("elevator goal position: " + elevatorGoal.position);
        System.out.println("elevator goal velocity: " + elevatorGoal.velocity);
        System.out.println("elbow goal position: " + elbowGoal.position);
        System.out.println("elbow goal velocity: " + elbowGoal.velocity);
        System.out.println("wrist goal position: " + wristGoal.position);
        System.out.println("wrist goal velocity: " + wristGoal.velocity);

        elevator.getElbowMotor().config_kP(0, elbowMotorP);
        elevator.getWristMotor().config_kP(0, wristMotorP);

        elevatorSetpoint = new TrapezoidProfile.State(elevator.getElevatorPosition(), elevator.getElevatorVelocity());
        elbowSetpoint = new TrapezoidProfile.State(elevator.getElbowPosition(), 0);
        wristSetpoint = new TrapezoidProfile.State(elevator.getWristPosition(), 0);
        
        elevatorProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(elevatorMaxVelocity, elevatorMaxAcceleration), elevatorGoal, elevatorSetpoint);
        elbowProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(elbowMaxVelocity, elbowMaxAcceleration), elbowGoal, elbowSetpoint);
        wristProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(wristMaxVelocity, wristMaxAcceleration), wristGoal, wristSetpoint);
    }

    @Override
    public void execute() {
        double timeChange = Timer.getFPGATimestamp() - startTime;
        elevator.setElevatorPosition(elevatorProfile.calculate(timeChange).position);
        elevator.setArmPosition(elbowProfile.calculate(timeChange).position, wristProfile.calculate(timeChange).position);
    }

    @Override
    public void end(boolean interrupted) {
        //elevator.stopArmMotors();
    }

    public boolean isFinished(){
        boolean temp = (Math.abs(elevator.getWristPosition() - wristGoal.position) <= allowableAngleError)
            && (Math.abs(elevator.getElbowPosition() - elbowGoal.position) <= allowableAngleError)
            && (Math.abs(elevator.getElevatorPosition() - elevatorGoal.position) <= allowableElevatorError);
        if(temp){
            System.out.println("command done");
            return true;
        }        
        return false;
    }
}