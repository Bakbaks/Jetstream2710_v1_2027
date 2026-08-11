package frc.robot.subsystems.Intake;

import frc.robot.Constants.IntakeConstants;

public class IntakeExtendoIOSim implements IntakeExtendoIO {
  private double inches, target;

  @Override
  public void updateInputs(IntakeExtendoIOInputs i) {
    inches += (target - inches) * .15;
    i.connected = true;
    i.positionInches = inches;
    i.positionRotations = inches / IntakeConstants.kInchesPerPinionRotation;
  }

  @Override
  public void setVoltage(double volts) {
    target = inches + volts / 12 * .1;
  }

  @Override
  public void setPositionInches(double value) {
    target = value;
  }

  @Override
  public void setZero() {
    inches = target = 0;
  }
}
