package frc.robot.subsystems.intakeextension;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO {
  @AutoLog
  class IntakeExtensionIOInputs {
    public boolean connected;
    public double positionInches;
    public double positionRotations;
    public double velocityRPM;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double temperatureCelsius;
  }

  default void updateInputs(IntakeExtensionIOInputs inputs) {}

  default void setPositionInches(double positionInches) {}

  default void setPositionRotations(double rotations) {}

  default void setVoltage(double volts) {}

  default void setEncoderZero() {}

  default void stop() {
    setVoltage(0.0);
  }
}
