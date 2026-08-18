package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
  private double extensionPositionInches;
  private double extensionTargetInches;
  private double rollerVolts;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    extensionPositionInches += (extensionTargetInches - extensionPositionInches) * 0.15;
    inputs.extensionConnected = true;
    inputs.rollerConnected = new boolean[] {true, true};
    inputs.extensionPositionInches = extensionPositionInches;
    inputs.rollerAppliedVolts = new double[] {rollerVolts, rollerVolts};
    inputs.rollerVelocityRPM =
        new double[] {
          rollerVolts / 12.0 * IntakeConstants.FREE_SPEED_RPM,
          rollerVolts / 12.0 * IntakeConstants.FREE_SPEED_RPM
        };
  }

  @Override
  public void setExtensionPositionInches(double positionInches) {
    extensionTargetInches = positionInches;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerVolts = volts;
  }

  @Override
  public void setExtensionEncoderZero() {
    extensionPositionInches = 0.0;
    extensionTargetInches = 0.0;
  }
}
