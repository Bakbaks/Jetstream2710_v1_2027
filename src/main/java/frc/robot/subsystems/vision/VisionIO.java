package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean[] connected = new boolean[3];
    public Pose2d[] estimatedPoses = new Pose2d[0];
    public double[] timestampsSeconds = new double[0];
    public double[] standardDeviations = new double[0];
    public int[] cameraIndices = new int[0];
    public int[] tagCounts = new int[3];
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
