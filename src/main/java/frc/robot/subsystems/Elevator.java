package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
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
    private Encoder elevatorEncoder;
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
        elevatorEncoder = new Encoder(0,1);


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

        bottomSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomSwitch.enableLimitSwitch(true);

        topSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        topSwitch.enableLimitSwitch(true);




    }
    /**
    *Changes the motor speed of the elevator motor
    *@param velocity desired motor speed
    */
    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
    }
    /**
    *Decides based on conditions how to run the elevator motor based on inputs from the top and bottom limit switches
    *@param targetPosition the position of the motor if the switches are not pressed
     */
    public void setElevatorPosition(double targetPosition)
    {
        if (topSwitch.isPressed()){
            elevatorMotor.set(0);
        }
        else if (bottomSwitch.isPressed()){
            elevatorMotor.set(0);
        }
        else {
            elevatorMotor.set(ControlMode.Position, targetPosition);
        }
    }

    /**
    * Returns true if top limit switch is enabled
    * @return Returns true if top switch is enabled
    */
    public boolean getTopSwitch() {
        return topSwitch.isLimitSwitchEnabled();
    }
    /**
    * Returns true if bottom limit switch is enabled
    * @return Returns true if bottom switch is enabled
    */
    public boolean getBottomSwitch() {
        return bottomSwitch.isLimitSwitchEnabled();
    }


    /**
    *Sets the variable so that the coneOrCube boolean is reversed
    */
    public void setConeOrCube()
    {
        coneOrCube = !coneOrCube;
    }
    /**
    * This code shows whether the variable coneOrCube is true or false.
    * @return Returns whether the variable coneOrCube is true or false
     */
    public boolean getConeOrCube()
    {
        return coneOrCube; 
    }
    /**
    *Sets arm motors to inputed positions
    *@param pos1 joint position for the elbow
    *@param pos2 joint position for wrist
     */
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

    /**
     * Resets elevator encoder
    */
    public void resetElevatorEncoder() {
        elevatorEncoder.reset();
    }
}