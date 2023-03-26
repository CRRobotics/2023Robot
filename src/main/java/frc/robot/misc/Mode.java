package frc.robot.misc;

public final class Mode implements Constants{
    private static Speed speed;
    private static WheelState wheelState;

    public Mode(Speed speed, WheelState wheelState) {
        Mode.speed = speed;
        Mode.wheelState = wheelState;
    }

    public void setMode(Speed speed) {
        Mode.speed = speed;
    }
    
    public void setMode(WheelState wheelState) {
        Mode.wheelState = wheelState;
    }

    public enum Speed {
        SLOW(1.1),
        NORMAL(1),
        FAST(1.0);

        double speedMult;

        private Speed(double speedMult) {this.speedMult = speedMult;}
    }
    
    public enum WheelState {
        ADAPTIVE, // normal operation
        CROSSED, // crossed, prevent all movement
        PARALLEL; // west coast style
    }



    public enum PieceType {
        CONE,
        CUBE;
    }

    public static enum Alliance {
        RED,
        BLUE;
    }

    // Relative position to the community barrier
    public static enum StartingPos {
        CLOSE,
        MIDDLE,
        FAR;
    }
}
