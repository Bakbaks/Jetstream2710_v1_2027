package frc.robot.shooting;

import org.wpilib.math.util.MathUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public final class ShotVerifier {
  private final RobotState robotState;
  private final Shooter shooter;

  public ShotVerifier(RobotState robotState, Shooter shooter) {
    this.robotState = robotState;
    this.shooter = shooter;
  }

  public boolean canFire(ShotSolution solution) {
    double headingError =
        MathUtil.angleModulus(
            solution.desiredHeading().minus(robotState.getPose().getRotation()).getRadians());
    var speeds = robotState.getFieldRelativeSpeeds();
    boolean headingReady = Math.abs(headingError) <= ShotConstants.HEADING_TOLERANCE_RADIANS;
    boolean movementReady =
        Math.hypot(speeds.vx, speeds.vy)
            <= ShotConstants.MAX_SHOOTING_SPEED_METERS_PER_SECOND;
    boolean canFire = solution.targetValid() && shooter.isReady() && headingReady && movementReady;
    Logger.recordOutput("Shot/TargetValid", solution.targetValid());
    Logger.recordOutput("Shot/HeadingReady", headingReady);
    Logger.recordOutput("Shot/MovementReady", movementReady);
    Logger.recordOutput("Shot/CanFire", canFire);
    return canFire;
  }
}
