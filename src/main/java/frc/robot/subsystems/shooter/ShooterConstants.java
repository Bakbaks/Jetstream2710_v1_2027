package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  public static final double IDLE_RPM = 0.0;
  public static final double REVERSE_RPM = -900.0;
  public static final double DEFAULT_SHOT_RPM = 1300.0;
  public static final double PASS_RPM = 1800.0;
  public static final double READY_TOLERANCE_RPM = 400.0;
  public static final double MIN_READY_TARGET_RPM = 100.0;
  public static final double MAX_ACCELERATION_RPM_PER_SECOND = 12000.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 80.0;

  // TODO: These gains are carried forward only to preserve behavior and require validation.
  public static final double VELOCITY_KP = 0.05;
  public static final double VELOCITY_KI = 0.0;
  public static final double VELOCITY_KD = 0.0;
  public static final double FREE_SPEED_RPM = 6000.0;

  private ShooterConstants() {}
}
