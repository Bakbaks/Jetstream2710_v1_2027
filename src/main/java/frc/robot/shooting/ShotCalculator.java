package frc.robot.shooting;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.RobotState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public final class ShotCalculator {
  private final RobotState robotState;
  private final ShotTable shotTable;

  public ShotCalculator(RobotState robotState, ShotTable shotTable) {
    this.robotState = robotState;
    this.shotTable = shotTable;
  }

  public ShotSolution calculate(Optional<Translation2d> targetPoint) {
    Pose2d robotPose = robotState.getPose();
    if (targetPoint.isEmpty()) {
      return ShotSolution.invalid(robotPose);
    }
    ChassisVelocities speeds = robotState.getFieldRelativeSpeeds();
    Pose2d latencyPose =
        new Pose2d(
            robotPose.getX() + speeds.vx * ShotConstants.SHOT_LATENCY_SECONDS,
            robotPose.getY() + speeds.vy * ShotConstants.SHOT_LATENCY_SECONDS,
            robotPose
                .getRotation()
                .plus(
                    Rotation2d.fromRadians(
                        speeds.omega * ShotConstants.SHOT_LATENCY_SECONDS)));
    Pose2d shooterExitPose = latencyPose.plus(ShotConstants.ROBOT_TO_SHOOTER_EXIT);
    Translation2d exitOffsetField =
        ShotConstants.ROBOT_TO_SHOOTER_EXIT.getTranslation().rotateBy(latencyPose.getRotation());
    Translation2d exitVelocity =
        new Translation2d(
            speeds.vx - speeds.omega * exitOffsetField.getY(),
            speeds.vy + speeds.omega * exitOffsetField.getX());
    double rawDistance = shooterExitPose.getTranslation().getDistance(targetPoint.get());
    double timeOfFlight = shotTable.getTimeOfFlightSeconds(rawDistance);
    Translation2d predictedExitPoint = shooterExitPose.getTranslation();
    double effectiveDistance = rawDistance;
    for (int i = 0; i < ShotConstants.PREDICTION_ITERATIONS; i++) {
      predictedExitPoint = shooterExitPose.getTranslation().plus(exitVelocity.times(timeOfFlight));
      effectiveDistance = predictedExitPoint.getDistance(targetPoint.get());
      timeOfFlight = shotTable.getTimeOfFlightSeconds(effectiveDistance);
    }
    Rotation2d heading =
        targetPoint.get().minus(predictedExitPoint).getAngle().plus(Rotation2d.k180deg);
    ShotSolution solution =
        new ShotSolution(
            true,
            shotTable.getRPM(effectiveDistance),
            heading,
            rawDistance,
            effectiveDistance,
            timeOfFlight,
            latencyPose,
            shooterExitPose,
            predictedExitPoint);
    Logger.recordOutput("Shot/Target", targetPoint.get());
    Logger.recordOutput("Shot/DesiredHeading", solution.desiredHeading());
    Logger.recordOutput("Shot/EffectiveDistanceMeters", solution.effectiveDistanceMeters());
    Logger.recordOutput("Shot/RPM", solution.shooterRPM());
    return solution;
  }
}
