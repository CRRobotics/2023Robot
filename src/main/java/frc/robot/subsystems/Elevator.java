package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator extends SubsystemBase{
    private Arm arm;
    private VictorSPX armMotor;
    private final PIDController elevatorPID;
    private final PIDController armPID;
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private Mechanism2d mechanism;
    private MechanismLigament2d wrist;
    private MechanismLigament2d wrist2;
    private MechanismLigament2d elevator;
    private SingleJointedArmSim armSim;
    private double volts;
    public Elevator(int motorID, int motorID2){
        arm = new Arm(new JointConfig(1, 1, 1, 1, DCMotor.getFalcon500(1)), 
        new JointConfig(1, 1, 1, 1, DCMotor.getFalcon500(1)));
        elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        armMotor = new VictorSPX(motorID2);
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);
        armPID = new PIDController(0.001, 0, 0); //TODO make these constants
        mechanism = new Mechanism2d(3,3);
        MechanismRoot2d root = mechanism.getRoot("root", 1, 1);
        elevator = root.append(new MechanismLigament2d("elavator", 1, 0));
        wrist = elevator.append(new MechanismLigament2d("wrist", 1, 1));
        wrist2 = wrist.append(new MechanismLigament2d("wrist2", 1, 1));
        SmartDashboard.putData("Mechanism", mechanism);
        elevatorEncoder = elevatorMotor.getEncoder();
        
        
        armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1),
          20, 0.0666, 0.2, 0, Math.PI * 2, 5, 
          true);

        armSim.setState(new Matrix<>(VecBuilder.fill(Math.PI * 3/4, 0)));
        SmartDashboard.putNumber("volts", volts);
    }

    @Override
    public void periodic() {
        volts = SmartDashboard.getNumber("volts", volts);
        double i = arm.calculate(VecBuilder.fill(1, 0)).get(0, 0);
        armSim.setInputVoltage(1.23);
        armSim.update(0.02);
        System.out.println(armSim.getAngleRads() * 180 / Math.PI);
        wrist.setAngle(armSim.getAngleRads() * 180 / Math.PI);
        wrist2.setAngle(wrist2.getAngle() + 1);
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