package frc.robot.subsystems.vision;

public class VisionIOSim implements VisionIO {
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = new boolean[] {true, true, true};
  }
}
