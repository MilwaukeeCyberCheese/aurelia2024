package frc.robot.utils;

/** PID constants used to create PID controllers */
public class PIDConstants {
  /** P */
  public final double kP;
  /** I */
  public final double kI;
  /** D */
  public final double kD;
  /**FF */
  public final double kFF;

  /**
   * Create a new PIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param kFF Feedforward
   */
  public PIDConstants(double kP, double kI, double kD, double kFF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;
  }

  /**
   * Create a new PIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   */
  public PIDConstants(double kP, double kI, double kD) {
    this(kP, kI, kD, 0.0);
  }
}
