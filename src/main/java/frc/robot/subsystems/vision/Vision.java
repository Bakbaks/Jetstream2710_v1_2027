package frc.robot.subsystems.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  @FunctionalInterface
  public interface EstimateConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations);
  }

  private final VisionIO io;
  private final EstimateConsumer estimateConsumer;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO io, EstimateConsumer estimateConsumer) {
    this.io = io;
    this.estimateConsumer = estimateConsumer;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    int count =
        Math.min(
            inputs.estimatedPoses.length,
            Math.min(inputs.timestampsSeconds.length, inputs.standardDeviations.length / 3));
    for (int i = 0; i < count; i++) {
      int offset = i * 3;
      estimateConsumer.accept(
          inputs.estimatedPoses[i],
          inputs.timestampsSeconds[i],
          VecBuilder.fill(
              inputs.standardDeviations[offset],
              inputs.standardDeviations[offset + 1],
              inputs.standardDeviations[offset + 2]));
    }
  }
}
