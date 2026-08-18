package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
  private double velocityRPM;
  private double targetRPM;
  private double appliedVolts;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    velocityRPM += (targetRPM - velocityRPM) * 0.12;
    inputs.connected = new boolean[] {true, true, true, true};
    inputs.velocityRPM = new double[] {velocityRPM, velocityRPM, velocityRPM, velocityRPM};
    inputs.appliedVolts = new double[] {appliedVolts, appliedVolts, appliedVolts, appliedVolts};
    inputs.supplyVolts = new double[] {12.0, 12.0, 12.0, 12.0};
    inputs.averageVelocityRPM = velocityRPM;
  }

  @Override
  public void setVelocityRPM(double rpm) {
    targetRPM = rpm;
    appliedVolts = Math.max(-12.0, Math.min(12.0, rpm / ShooterConstants.FREE_SPEED_RPM * 12.0));
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    targetRPM = volts / 12.0 * ShooterConstants.FREE_SPEED_RPM;
  }
}
