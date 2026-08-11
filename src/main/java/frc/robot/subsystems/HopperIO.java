package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  class HopperIOInputs {
    public boolean[] connected = new boolean[4];
    public double[] velocityRPM = new double[4];
    public double[] appliedVolts = new double[4];
    public double[] supplyCurrentAmps = new double[4];
    public double[] statorCurrentAmps = new double[4];
    public double[] temperatureCelsius = new double[4];
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setVelocityRPM(double floorRPM, double feederRPM) {}

  default void setVoltage(double floorVolts, double feederVolts) {}

  default void stop() {
    setVoltage(0.0, 0.0);
  }
}
