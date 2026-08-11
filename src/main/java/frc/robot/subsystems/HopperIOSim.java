package frc.robot.subsystems;

public class HopperIOSim implements HopperIO {
  private double floorRPM, feederRPM;

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.connected = new boolean[] {true, true, true, true};
    inputs.velocityRPM = new double[] {floorRPM, feederRPM, floorRPM, feederRPM};
  }

  @Override
  public void setVelocityRPM(double floor, double feeder) {
    floorRPM = floor;
    feederRPM = feeder;
  }

  @Override
  public void setVoltage(double floor, double feeder) {
    floorRPM = floor / 12.0 * 6000.0;
    feederRPM = feeder / 12.0 * 6000.0;
  }
}
