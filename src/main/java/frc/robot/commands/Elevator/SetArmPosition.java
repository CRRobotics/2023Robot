package frc.robot.commands.Elevator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Elevator;

public class SetArmPosition extends CommandBase implements Constants.Elevator {
    private Elevator elevator;
    private double elbowPosition;
    private double wristPosition;
    private double elevatorPosition;
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

    public SetArmPosition(Elevator elevator, double elevatorPosition, double elbowPosition, double wristPosition) {
        this.elevator = elevator;
        this. elevatorPosition = elevatorPosition;
        this.elbowPosition = elbowPosition * (Math.PI / 180);
        this.wristPosition = wristPosition * (Math.PI / 180);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        elevator.setPosition();

        elevatorGoal = new TrapezoidProfile.State(elevatorPosition, 0);
        elbowGoal = new TrapezoidProfile.State(elbowPosition, 0);
        wristGoal = new TrapezoidProfile.State(wristPosition, 0);

        elevator.getElbowMotor().config_kP(0, SmartDashboard.getNumber("elbow/elbow P", 0));
        elevator.getElbowMotor().config_kI(0, SmartDashboard.getNumber("elbow/elbow I", 0));
        elevator.getElbowMotor().config_kD(0, SmartDashboard.getNumber("elbow/elbow D", 0));

        elevator.getWristMotor().config_kP(0, SmartDashboard.getNumber("wrist/wrist P", 0.08));
        elevator.getWristMotor().config_kI(0, SmartDashboard.getNumber("wrist/wrist I", 0));
        elevator.getWristMotor() .config_kD(0, SmartDashboard.getNumber("wrist/wrist D", 0));

        elevatorSetpoint = new TrapezoidProfile.State(elevator.getElevatorPosition(), 0);
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