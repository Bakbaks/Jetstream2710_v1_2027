package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtendoIO {
  @AutoLog
  class IntakeExtendoIOInputs {
    public boolean connected;
    public double positionRotations;
    public double positionInches;
    public double velocityRPM;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double temperatureCelsius;
  }

  default void updateInputs(IntakeExtendoIOInputs i) {}

  default void setVoltage(double volts) {}

  default void setPositionInches(double inches) {}

  default void setZero() {}
}
