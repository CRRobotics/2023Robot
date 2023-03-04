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
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Contains the arm and elevator assemblies
 */
public class Elevator extends SubsystemBase implements Constants.Elevator {
    private CANSparkMax elevatorMotor; // Linear axis
    private Encoder elevatorEncoder;
    private SparkMaxLimitSwitch topSwitch; //Limit switch on linear axis
    private SparkMaxLimitSwitch bottomSwitch; //Limit switch on linear axis

    private PIDController elevatorPID;

    private TalonFX elbowMotor; //Works with armMotor2
    private TalonFXConfiguration elbowConfig;
    private TalonFX wristMotor; //Works with armMotor1
    private TalonFXConfiguration wristConfig;


    private boolean coneOrCube; //True for cone, false for cube

    /**
     * Constructs an <code>Elevator</code> using motor ID's in <code>Contants.java</code>
     */
    public Elevator() {
        coneOrCube = true;

        elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(true);
        elevatorMotor.setSmartCurrentLimit(elevatorCurrentLimit);
        elevatorEncoder = new Encoder(0,1);
        elevatorEncoder.setDistancePerPulse(1);
        elevatorEncoder.reset();

        elevatorPID = new PIDController(0,0,0);


        elbowMotor = new TalonFX(elbowMotorID);
        elbowMotor.configFactoryDefault();
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowConfig = new TalonFXConfiguration();
        elbowConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        elbowMotor.setSelectedSensorPosition(0);
        elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, elbowMotorCurrentLimit, elbowMotorCurrentLimit, 0));
        elbowConfig.slot0.allowableClosedloopError = 0;
        elbowMotor.selectProfileSlot(0, 0);
        elbowConfig.neutralDeadband = 0;
        elbowConfig.slot0.closedLoopPeakOutput = 0.8;
        elbowMotor.configAllSettings(elbowConfig);

        wristMotor = new TalonFX(wristMotorID);
        wristMotor.configFactoryDefault(100);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristConfig = new TalonFXConfiguration();
        wristConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        wristMotor.setSelectedSensorPosition(0);
        wristMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, wristMotorCurrentLimit, wristMotorCurrentLimit, 0));
        wristConfig.slot0.allowableClosedloopError = 0;
        wristMotor.selectProfileSlot(0, 0);
        wristConfig.neutralDeadband = 0;
        wristConfig.slot0.closedLoopPeakOutput = 1;
        wristMotor.configAllSettings(wristConfig);
        

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
        // elevatorMotor.set(velocity * 0.1);
        // SmartDashboard.putNumber("elevator/percent output", velocity * 0.1);
    }
    /**
    *Decides based on conditions how to run the elevator motor based on inputs from the top and bottom limit switches
    *@param targetPosition the position of the motor if the switches are not pressed
     */
    public void setElevatorPosition(double targetPosition)
    {
        elevatorPID.setSetpoint(SmartDashboard.getNumber("elevator/elevator setpoint", 0) * elevatorTicksPerMeter);
        elevatorMotor.set(SmartDashboard.getNumber("elevator/kg", 0.03)
            + elevatorPID.calculate(elevatorEncoder.getDistance()));
    }

    
    /**
    *Sets arm motors to inputed positions
    *@param pos1 joint position for the elbow
    *@param pos2 joint position for wrist
     */
    public void setArmMotors(double pos1, double pos2){
        elbowMotor.config_kP(0, SmartDashboard.getNumber("elbow/elbow P", 0));
        elbowMotor.config_kI(0, SmartDashboard.getNumber("elbow/elbow I", 0));
        elbowMotor.config_kD(0, SmartDashboard.getNumber("elbow/elbow D", 0));

        wristMotor.config_kP(0, SmartDashboard.getNumber("wrist/wrist P", 0.08));
        wristMotor.config_kI(0, SmartDashboard.getNumber("wrist/wrist I", 0));
        wristMotor.config_kD(0, SmartDashboard.getNumber("wrist/wrist D", 0));

        elbowMotor.set(TalonFXControlMode.Position,
            SmartDashboard.getNumber("elbow/elbow setpoint", 0) * elbowTicksPerDegree,
            DemandType.ArbitraryFeedForward,
            SmartDashboard.getNumber("elbow/kg", 0.01)
            * Math.cos(elbowMotor.getSelectedSensorPosition() / (elbowTicksPerRadian)));

        wristMotor.set(TalonFXControlMode.Position,
        SmartDashboard.getNumber("wrist/wrist setpoint", 0) * wristTicksPerDegree,
            DemandType.ArbitraryFeedForward,
            SmartDashboard.getNumber("wrist/kg", 0.01)
            * Math.cos(wristMotor.getSelectedSensorPosition() / (wristTicksPerRadian)));
    }

    public void stopArmMotors() {
        elbowMotor.set(ControlMode.PercentOutput, 0);
        wristMotor.set(ControlMode.PercentOutput, 0);
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elbow/elbow position", elbowMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("elbow/elbow voltage", elbowMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("elbow/elbow position degrees", elbowMotor.getSelectedSensorPosition() / 189);

        SmartDashboard.putNumber("wrist/wrist position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("wrist/wrist voltage", wristMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("wrist/wrist position degrees", wristMotor.getSelectedSensorPosition() / 189);

        SmartDashboard.putNumber("elevator/elevator position", elevatorEncoder.getDistance());
        SmartDashboard.putNumber("elevator/elevator position meters", elevatorEncoder.getDistance() * elevatorTicksPerMeter);
    }

    /**
     * Resets elevator encoder
    */
    public void resetElevatorEncoder() {
        elevatorEncoder.reset();
        elbowMotor.setSelectedSensorPosition(0);
        wristMotor.setSelectedSensorPosition(0);
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
}