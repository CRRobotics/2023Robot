package frc.robot.misc;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double-jointed arm.
 * <a>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060</a>
 */
public class ArmMath {
  private static final double g = 9.80665;

  /**
   * Class that stores all the parameters as pivs
   * with getters and setters
   */
  public class JointConfig {
    private final double mass;
    private final double length;
    private final double moi;
    private final double cgRadius;
    private final DCMotor motor;
    /**
     * Creates an object that saves all the joint variables in one place.
     * @param mass The mass of the arm.
     * @param length Length of the arm
     * @param moi Moment of inertia
     * @param cgRadius Radius of the center of gravity.
     * @param motor The motor used to move the arm.
     */
    public JointConfig(double mass, double length, double moi, double cgRadius, DCMotor motor) {
      this.mass = mass;
      this.length = length;
      this.moi = moi;
      this.cgRadius = cgRadius;
      this.motor = motor;
    }

    /**
     * Returns the mass of the arm.
     * @return the mass of the arm.
     */
    public double mass() {
      return mass;
    }

    /**
     * Returns the length of the arm attached to the motor.
     * @return the length of the arm attached to the motor.
     */
    public double length() {
      return length;
    }

    /**
     * Returns the moment of inertia.
     * @return the moment of inertia.
     */
    public double moi() {
      return moi;
    }

    /**
     * Returns the radius of the center of gravity of the arm.
     * @return the radius of the center of gravity of the arm.
     */
    public double cgRadius() {
      return cgRadius;
    }

    /**
     * Returns the <code>DCMotor</code> that the arm uses.
     * @return the <code>DCMotor</code> that the arm uses.
     */
    public DCMotor motor() {
      return motor;
    }
  }


  private final JointConfig joint_1;
  private final JointConfig joint_2;

  public ArmMath(JointConfig joint_1, JointConfig joint_2) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
  }

  /**
   * Calculates the voltage necessary to maintain the two motors' positions
   * @param position The current position
   * @return The feedforward term for each motor
   */
  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  /**
   * Calculates the amount of voltage necessary to move the two arm motors based on current position, velocity, and acceleration
   * @param position Current position of the two motors
   * @param velocity Current velocity
   * @param acceleration Current acceleration
   * @return The voltage necessary
   */
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