package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean extensionConnected;
    public boolean[] rollerConnected = new boolean[2];
    public double extensionPositionInches;
    public double extensionVelocityRPM;
    public double extensionAppliedVolts;
    public double extensionSupplyCurrentAmps;
    public double extensionStatorCurrentAmps;
    public double extensionTemperatureCelsius;
    public double[] rollerVelocityRPM = new double[2];
    public double[] rollerAppliedVolts = new double[2];
    public double[] rollerSupplyCurrentAmps = new double[2];
    public double[] rollerStatorCurrentAmps = new double[2];
    public double[] rollerTemperatureCelsius = new double[2];
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setExtensionPositionInches(double positionInches) {}

  default void setRollerVoltage(double volts) {}

  default void setExtensionEncoderZero() {}
}
