package frc.robot.subsystems.intakerollers;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private double appliedVolts;

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.connected = new boolean[] {true, true};
    inputs.appliedVolts = new double[] {appliedVolts, appliedVolts};
    inputs.velocityRPM =
        new double[] {
          appliedVolts / 12.0 * IntakeRollersConstants.FREE_SPEED_RPM,
          appliedVolts / 12.0 * IntakeRollersConstants.FREE_SPEED_RPM
        };
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
