package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Contains the arm and elevator assemblies
 */
public class Elevator extends SubsystemBase implements Constants.Elevator {
    private CANSparkMax elevatorMotor; // Linear axis
    private RelativeEncoder elevatorEncoder;
    private TalonFX elbowMotor; //Works with armMotor2
    private TalonFX wristMotor; //Works with armMotor1

    private SparkMaxLimitSwitch topSwitch; //Limit switch on linear axis
    private SparkMaxLimitSwitch bottomSwitch; //Limit switch on linear axis

    private boolean coneOrCube; //True for cone, false for cube

    /**
     * Constructs an <code>Elevator</code> using motor ID's in <code>Contants.java</code>
     */
    public Elevator() {
        coneOrCube = true;
        elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(elevatorCurrentLimit);
        elevatorEncoder = elevatorMotor.getEncoder();

        elbowMotor = new TalonFX(elbowMotorID);
        elbowMotor.configFactoryDefault();
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elbowMotor.setSelectedSensorPosition(0);
        elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, elbowMotorCurrentLimit, elbowMotorCurrentLimit, 0));
        elbowMotor.configVoltageCompSaturation(elbowMotorVoltageLimit);

        wristMotor = new TalonFX(wristMotorID);
        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        wristMotor.setSelectedSensorPosition(0);
        wristMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, wristMotorCurrentLimit, wristMotorCurrentLimit, 0));
        wristMotor.configVoltageCompSaturation(wristMotorVoltageLimit);

        // not mounted yet
        // bottomSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        // bottomSwitch.enableLimitSwitch(true);

        // topSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        // topSwitch.enableLimitSwitch(true);




    }

    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
    }

    public void setElevatorPosition(double targetPosition)
    {
        // if (topSwitch.isPressed()){
        //     elevatorMotor.set(0);
        // }
        // else if (bottomSwitch.isPressed()){
        //     elevatorMotor.set(0);
        // }

    }


    public SparkMaxLimitSwitch getTopSwitch() {
        return topSwitch;
    }
    public SparkMaxLimitSwitch getBottomSwitch() {
        return bottomSwitch;
    }



    public void setConeOrCube()
    {
        if (coneOrCube) coneOrCube = false;
        else coneOrCube = true;
    }

    public boolean getConeOrCube()
    {
        return coneOrCube; 
    }

    public void setArmMotors(double pos1, double pos2){
        elbowMotor.config_kP(0, SmartDashboard.getNumber("elbow P", 0));
        elbowMotor.config_kI(0, SmartDashboard.getNumber("elbow I", 0));
        elbowMotor.config_kD(0, SmartDashboard.getNumber("elbow D", 0));

        wristMotor.config_kP(0, SmartDashboard.getNumber("wrist P", 0));
        wristMotor.config_kI(0, SmartDashboard.getNumber("wrist I", 0));
        wristMotor.config_kD(0, SmartDashboard.getNumber("wrist D", 0));

        wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        wristMotor.set(ControlMode.Position, pos2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elbow position", elbowMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("wrist position", elbowMotor.getSelectedSensorPosition());
    }
}