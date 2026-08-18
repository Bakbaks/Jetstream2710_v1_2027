package frc.robot.shooting;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

public final class ShotConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final int RED_TARGET_TAG_ID = 10;
  public static final int BLUE_TARGET_TAG_ID = 26;
  public static final Transform2d TAG_TO_TARGET = new Transform2d(-0.62, 0.0, Rotation2d.kZero);
  public static final Transform2d ROBOT_TO_SHOOTER_EXIT =
      new Transform2d(-0.1435885114, 0.0, Rotation2d.kZero);
  public static final double SHOT_LATENCY_SECONDS = 0.34;
  public static final int PREDICTION_ITERATIONS = 2;
  public static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(1.0);
  public static final double MAX_SHOOTING_SPEED_METERS_PER_SECOND = 1.0;
  public static final double DEFAULT_TIME_OF_FLIGHT_SECONDS = 0.28;

  private ShotConstants() {}
}
