package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public boolean[] connected = new boolean[4];
    public double[] velocityRPM = new double[4];
    public double[] appliedVolts = new double[4];
    public double[] supplyCurrentAmps = new double[4];
    public double[] statorCurrentAmps = new double[4];
    public double[] temperatureCelsius = new double[4];
    public double averageVelocityRPM;
    public boolean closedLoop;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVelocityRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }

  default void setFeedforward(double kS, double kV, double kA) {}

  default double getFeedforwardKS() {
    return 0.0;
  }

  default double getFeedforwardKV() {
    return 0.0;
  }

  default double getFeedforwardKA() {
    return 0.0;
  }
}
