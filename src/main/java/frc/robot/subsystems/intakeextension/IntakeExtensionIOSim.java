package frc.robot.subsystems.intakeextension;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  private double positionRotations;
  private double targetRotations;
  private double appliedVolts;
  private boolean openLoop;

  @Override
  public void updateInputs(IntakeExtensionIOInputs inputs) {
    if (openLoop) {
      positionRotations += appliedVolts / 12.0 * 0.08;
    } else {
      positionRotations += (targetRotations - positionRotations) * 0.15;
    }
    inputs.connected = true;
    inputs.positionRotations = positionRotations;
    inputs.positionInches =
        positionRotations * IntakeExtensionConstants.INCHES_PER_PINION_ROTATION;
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setPositionInches(double positionInches) {
    openLoop = false;
    targetRotations = positionInches / IntakeExtensionConstants.INCHES_PER_PINION_ROTATION;
  }

  @Override
  public void setPositionRotations(double rotations) {
    openLoop = false;
    targetRotations = rotations;
  }

  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    appliedVolts = volts;
  }

  @Override
  public void setEncoderZero() {
    positionRotations = 0.0;
    targetRotations = 0.0;
  }
}
