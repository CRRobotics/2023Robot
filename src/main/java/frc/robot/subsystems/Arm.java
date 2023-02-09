package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 */
public class Arm {
  private static final double g = 9.80665;

  /*basically creates a class that stores all the parameters as pivs
    with getters and setters
  */
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

  private final JointConfig joint_1;
  private final JointConfig joint_2;

  public Arm(JointConfig joint_1, JointConfig joint_2) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
  }

  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var M = new Matrix<>(N2.instance, N2.instance);
    var C = new Matrix<>(N2.instance, N2.instance);
    var Tg = new Matrix<>(N2.instance, N1.instance);

    M.set(
        0,
        0,
        joint_1.mass() * Math.pow(joint_1.cgRadius(), 2.0)
            + joint_2.mass() * (Math.pow(joint_1.length(), 2.0) + Math.pow(joint_2.cgRadius(), 2.0))
            + joint_1.moi()
            + joint_2.moi()
            + 2
                * joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0) + joint_2.moi());
    C.set(
        0,
        0,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (joint_1.mass() * joint_1.cgRadius() + joint_2.mass() * joint_1.length())
                * g
                * Math.cos(position.get(0, 0))
            + joint_2.mass()
                * joint_2.cgRadius()
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_2.mass()
            * joint_2.cgRadius()
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        joint_1.motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_2.motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}