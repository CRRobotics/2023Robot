package frc.robot.misc;

import edu.wpi.first.wpilibj.DriverStation;

public final class Mode implements Constants {
    //#region STATE TYPES
    // Constant
    private static ControllMode controllMode;
    private static Alliance alliance;
    private static StartingPose startingPose;
    // Dynamic
    private static OperationState operationState;
    private static Speed speed;
    private static WheelState wheelState;
    private static PieceType pieceType;
    //#endregion STATE TYPES
    
    //#region CONSTRUCTORS
    /**
     * Default values
     * speed = NORMAL
     * wheelState = ADAPTIVE
     * pieceType = NA
     * alliance = NA
     * startingPose = NA
     */
    public Mode() {
        // Constant
        Mode.controllMode = ControllMode.ROBOT_RELATIVE;
        Mode.alliance = Alliance.NA;
        Mode.startingPose = StartingPose.NA;
        // Dynamic
        Mode.operationState = OperationState.COMPETITION;
        Mode.speed = Speed.NORMAL;;
        Mode.wheelState = WheelState.SWERVE;
        Mode.pieceType = PieceType.NA;
    }

    /**
     * Default values for everything except allience and starting position
     */
    public Mode(DriverStation.Alliance alliance, int startingPose, ControllMode controllMode) {
        this();
        Mode.alliance = (alliance == DriverStation.Alliance.Red)? Mode.Alliance.RED:Mode.Alliance.BLUE;
        Mode.controllMode = controllMode;

        switch(startingPose) {
            case 1 :
                Mode.startingPose = Mode.StartingPose.CLOSE;
                break;
            case 2 :
                Mode.startingPose = Mode.StartingPose.MIDDLE;
                break;
            case 3 :
                Mode.startingPose = Mode.StartingPose.FAR;
                break;
        }
    }
    //#endregion CONSTRUCTORS

    //#region STATES

    //#region Constant states
    
    /**
     * Dictates the function and integration of odometry and visions into driving
     */
    public enum ControllMode {
        FIELD_RELATIVE_VISIONS, // Odometry + visions
        FIELD_RELATIVE_NO_VISIONS, // Only odometry
        ROBOT_RELATIVE; // No odometry needed
    }
    
    /**
     * Alliance color
     */
    public static enum Alliance {
        RED,
        BLUE,
        NA; // No alliance active
    }
    
    /**
     * Relative position to the community barrier
     */
    public static enum StartingPose {
        CLOSE, // Right on red, left on blue
        MIDDLE, // Center on both alliances
        FAR, // Left on red, right on blue
        NA; // No starting pose active
    }

    //#endregion Constant states

    //#region Dynamic states

    /**
     * Changes overall operation speed and debugging information
     */
    public enum OperationState {
        COMPETITION, // Normal, maximum speed
        DEMO, // For inexperienced drivers, demos, etc
        DEBUG; // Really really slow, very safe :)
    }

    /**
     * Driving speed multiplier for enhanced precision
     * 0 < speedMult <= 1
     */
    public enum Speed {
        SLOW(0.6),
        NORMAL(0.8),
        FAST(1);

        double speedMult;

        private Speed(double speedMult) {this.speedMult = speedMult;}
    }
    
    /**
     * Wheel configuration
     */
    public enum WheelState {
        SWERVE, // Normal swerve operation
        INLINE, // Simulate east-coast operation
        CROSSED; // Rotate swerve wheels into an "X" formation
    }

    /**
     * Game piece interacting with the end effector
     */
    public enum PieceType {
        CONE,
        CUBE,
        NA; // No piece active
    }

    //#endregion Dynamic states

    //#endregion STATES

    //#region GETTERS

    //#region Constant states
    public static ControllMode getControllMode() {return controllMode;}
    public static Alliance getAlliance() {return alliance;}
    public static StartingPose getStartingPose() {return startingPose;}
    //#endregion GET Constant states

    //#region Dynamic states
    public static OperationState getOperationState() {return operationState;}
    public static Speed getSpeed() {return speed;}
    public static WheelState getWheelState() {return wheelState;}
    public static PieceType getPieceType() {return pieceType;}
    //#endregion GET Dynamic states

    //#endregion GETTERS

    //#region SETTERS
    
    //#region Constant states
    private void setMode(ControllMode controllMode) throws Exception {throw new Exception("Cant set mode: mode type should be constant");}
    private void setMode(Alliance alliance) throws Exception {throw new Exception("Cant set mode: mode type should be constant");}
    private void setMode(StartingPose startingPose) throws Exception {throw new Exception("Cant set mode: mode type should be constant");}
    //#endregion SET Constant states

    //#region Dynamic states
    public void setMode(OperationState operationState) {Mode.operationState = operationState;}
    public void setMode(Speed speed) {Mode.speed = speed;}
    public void setMode(WheelState wheelState) {Mode.wheelState = wheelState;}
    public void setMode(PieceType pieceType) {Mode.pieceType = pieceType;}
    //#endregion SET Dynamic states

    //#endregion SETTERS
}
