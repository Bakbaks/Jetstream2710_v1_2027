package frc.robot.util;

public final class DriveInput {
  private DriveInput() {}

  public static double exponential(double input, double base) {
    if (input < -1.0 || input > 1.0) {
      throw new IllegalArgumentException("Drive input must be between -1 and 1");
    }
    return input >= 0.0 ? Math.pow(base, input) - 1.0 : -Math.pow(1.0 / base, input) + 1.0;
  }
}
