package frc.robot.subsystems.intakerollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeRollersIOInputs {
    public boolean[] connected = new boolean[2];
    public double[] velocityRPM = new double[2];
    public double[] appliedVolts = new double[2];
    public double[] supplyCurrentAmps = new double[2];
    public double[] statorCurrentAmps = new double[2];
    public double[] temperatureCelsius = new double[2];
  }

  default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
