package frc.robot.shooting;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public record ShotSolution(
    boolean targetValid,
    double shooterRPM,
    Rotation2d desiredHeading,
    double rawDistanceMeters,
    double effectiveDistanceMeters,
    double timeOfFlightSeconds,
    Pose2d latencyCompensatedRobotPose,
    Pose2d shooterExitPose,
    Translation2d predictedExitPoint) {

  public static ShotSolution invalid(Pose2d robotPose) {
    return new ShotSolution(
        false,
        0.0,
        robotPose.getRotation(),
        0.0,
        0.0,
        0.0,
        robotPose,
        robotPose,
        robotPose.getTranslation());
  }
}
