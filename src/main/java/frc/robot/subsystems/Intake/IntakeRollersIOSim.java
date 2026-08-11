package frc.robot.subsystems.Intake;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private double volts;

  @Override
  public void updateInputs(IntakeRollersIOInputs i) {
    i.connected = new boolean[] {true, true};
    i.appliedVolts = new double[] {volts, volts};
    i.velocityRPM = new double[] {volts / 12 * 6000, volts / 12 * 6000};
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
