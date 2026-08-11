package frc.robot.subsystems;

public class FlywheelIOSim implements FlywheelIO {
  private double velocityRPM;
  private double targetRPM;
  private boolean closedLoop;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    velocityRPM += (targetRPM - velocityRPM) * 0.12;
    inputs.connected = new boolean[] {true, true, true, true};
    inputs.velocityRPM = new double[] {velocityRPM, velocityRPM, velocityRPM, velocityRPM};
    inputs.appliedVolts =
        new double[] {
          closedLoop ? 12.0 : 0.0,
          closedLoop ? 12.0 : 0.0,
          closedLoop ? 12.0 : 0.0,
          closedLoop ? 12.0 : 0.0
        };
    inputs.averageVelocityRPM = velocityRPM;
    inputs.closedLoop = closedLoop;
  }

  @Override
  public void setVelocityRPM(double rpm) {
    targetRPM = rpm;
    closedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    targetRPM = volts / 12.0 * 6000.0;
    closedLoop = false;
  }
}
