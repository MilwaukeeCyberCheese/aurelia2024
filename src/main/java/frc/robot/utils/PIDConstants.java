package frc.robot.utils;

/** PID constants used to create PID controllers */
public class PIDConstants {
  /** P */
  public final double kP;
  /** I */
  public final double kI;
  /** D */
  public final double kD;
  /** FF */
  public final double kFF;

  /**
   * Create a new PIDConstants object
   *
   * @param kP  P
   * @param kI  I
   * @param kD  D
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

  /**
   * Return a string of all the values
   */
  public String toString() {
    return "P: " + kP + " I: " + kI + " D: " + kD + " FF: " + kFF;
  }

  /**
   * Create a new PIDConstants object from string
   */
  public static PIDConstants parsePIDConstants(String str) {
    String[] parts = str.split(" ");
    try {
      return new PIDConstants(Double.parseDouble(parts[1]), Double.parseDouble(parts[3]), Double.parseDouble(parts[5]),
          Double.parseDouble(parts[7]));
    } catch (Exception e) {
      throw new IllegalArgumentException(
          "Invalid PIDConstants string format. Must be 'P: <double> I: <double> D: <double> FF: <double>'");
    }
  }
}