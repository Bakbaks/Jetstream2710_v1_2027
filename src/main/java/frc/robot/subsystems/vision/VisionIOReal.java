package frc.robot.subsystems.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
  private final PhotonCamera[] cameras = new PhotonCamera[VisionConstants.CAMERA_NAMES.length];
  private final PhotonPoseEstimator[] estimators =
      new PhotonPoseEstimator[VisionConstants.CAMERA_NAMES.length];

  public VisionIOReal() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new PhotonCamera(VisionConstants.CAMERA_NAMES[i]);
      estimators[i] =
          new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT, VisionConstants.ROBOT_TO_CAMERAS[i]);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<Pose2d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    List<Double> standardDeviations = new ArrayList<>();
    List<Integer> cameraIndices = new ArrayList<>();
    for (int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++) {
      inputs.connected[cameraIndex] = cameras[cameraIndex].isConnected();
      inputs.tagCounts[cameraIndex] = 0;
      for (var result : cameras[cameraIndex].getAllUnreadResults()) {
        inputs.tagCounts[cameraIndex] = result.getTargets().size();
        var estimate = estimators[cameraIndex].estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
          estimate = estimators[cameraIndex].estimateLowestAmbiguityPose(result);
        }
        if (estimate.isEmpty()) {
          continue;
        }
        Matrix<N3, N1> deviations =
            calculateStandardDeviations(
                estimators[cameraIndex], estimate.get(), result.getTargets());
        poses.add(estimate.get().estimatedPose.toPose2d());
        timestamps.add(estimate.get().timestampSeconds);
        cameraIndices.add(cameraIndex);
        standardDeviations.add(deviations.get(0, 0));
        standardDeviations.add(deviations.get(1, 0));
        standardDeviations.add(deviations.get(2, 0));
      }
    }
    inputs.estimatedPoses = poses.toArray(Pose2d[]::new);
    inputs.timestampsSeconds = timestamps.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.standardDeviations =
        standardDeviations.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.cameraIndices = cameraIndices.stream().mapToInt(Integer::intValue).toArray();
  }

  private static Matrix<N3, N1> calculateStandardDeviations(
      PhotonPoseEstimator estimator,
      EstimatedRobotPose estimatedPose,
      List<PhotonTrackedTarget> targets) {
    int validTags = 0;
    double averageDistance = 0.0;
    for (var target : targets) {
      var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      validTags++;
      averageDistance +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }
    if (validTags == 0) {
      return VisionConstants.SINGLE_TAG_STD_DEVS;
    }
    averageDistance /= validTags;
    if (validTags == 1 && averageDistance > VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    Matrix<N3, N1> base =
        validTags > 1 ? VisionConstants.MULTI_TAG_STD_DEVS : VisionConstants.SINGLE_TAG_STD_DEVS;
    return base.times(1.0 + averageDistance * averageDistance / 30.0);
  }
}
