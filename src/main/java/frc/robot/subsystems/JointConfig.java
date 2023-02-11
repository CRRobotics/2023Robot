package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;

public class JointConfig {
    private final double mass;
    private final double length;
    private final double moi;
    private final double cgRadius;
    private final DCMotor motor;

    public JointConfig(double mass, double length, double moi, double cgRadius, DCMotor motor) {
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.cgRadius = cgRadius;
        this.motor = motor;
    }

    public double mass() {
        return mass;
    }

    public double length() {
        return length;
    }

    public double moi() {
        return moi;
    }
    
    public double cgRadius() {
        return cgRadius;
    }

    public DCMotor motor() {
        return motor;
    }
}
