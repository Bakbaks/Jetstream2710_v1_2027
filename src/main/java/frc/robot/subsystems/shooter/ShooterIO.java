package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public boolean[] connected = new boolean[4];
    public double[] velocityRPM = new double[4];
    public double[] appliedVolts = new double[4];
    public double[] supplyVolts = new double[4];
    public double[] supplyCurrentAmps = new double[4];
    public double[] statorCurrentAmps = new double[4];
    public double[] temperatureCelsius = new double[4];
    public double averageVelocityRPM;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocityRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
