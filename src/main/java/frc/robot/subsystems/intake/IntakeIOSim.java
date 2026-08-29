package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
  private double extensionPositionRotations;
  private double extensionTargetRotations;
  private double extensionVolts;
  private boolean extensionOpenLoop;
  private double rollerVolts;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (extensionOpenLoop) {
      extensionPositionRotations += extensionVolts / 12.0 * 0.08;
    } else {
      extensionPositionRotations +=
          (extensionTargetRotations - extensionPositionRotations) * 0.15;
    }
    inputs.extensionConnected = true;
    inputs.rollerConnected = new boolean[] {true, true};
    inputs.extensionPositionRotations = extensionPositionRotations;
    inputs.extensionPositionInches =
        extensionPositionRotations * IntakeConstants.INCHES_PER_PINION_ROTATION;
    inputs.extensionAppliedVolts = extensionVolts;
    inputs.rollerAppliedVolts = new double[] {rollerVolts, rollerVolts};
    inputs.rollerVelocityRPM =
        new double[] {
          rollerVolts / 12.0 * IntakeConstants.FREE_SPEED_RPM,
          rollerVolts / 12.0 * IntakeConstants.FREE_SPEED_RPM
        };
  }

  @Override
  public void setExtensionPositionInches(double positionInches) {
    extensionOpenLoop = false;
    extensionTargetRotations = positionInches / IntakeConstants.INCHES_PER_PINION_ROTATION;
  }

  @Override
  public void setExtensionPositionRotations(double rotations) {
    extensionOpenLoop = false;
    extensionTargetRotations = rotations;
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionOpenLoop = true;
    extensionVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerVolts = volts;
  }

  @Override
  public void setExtensionEncoderZero() {
    extensionPositionRotations = 0.0;
    extensionTargetRotations = 0.0;
  }
}
