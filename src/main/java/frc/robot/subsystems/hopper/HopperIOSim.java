package frc.robot.subsystems.hopper;

public class HopperIOSim implements HopperIO {
  private double floorRPM;
  private double feederRPM;

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.connected = new boolean[] {true, true, true, true};
    inputs.velocityRPM = new double[] {floorRPM, feederRPM, floorRPM, feederRPM};
    inputs.supplyVolts = new double[] {12.0, 12.0, 12.0, 12.0};
  }

  @Override
  public void setVelocityRPM(double floorRPM, double feederRPM) {
    this.floorRPM = floorRPM;
    this.feederRPM = feederRPM;
  }
}
