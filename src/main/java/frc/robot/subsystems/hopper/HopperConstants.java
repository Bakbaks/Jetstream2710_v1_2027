package frc.robot.subsystems.hopper;

public final class HopperConstants {
  public static final double FEED_FLOOR_RPM = 5000.0;
  public static final double FEED_FEEDER_RPM = 5800.0;
  public static final double REVERSE_RPM = -900.0;
  public static final double UNJAM_RPM = -900.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;

  // TODO: Unvalidated values retained from the existing robot.
  public static final double VELOCITY_KP = 0.05;
  public static final double VELOCITY_KI = 0.0;
  public static final double VELOCITY_KD = 0.0;
  public static final double FREE_SPEED_RPM = 6000.0;

  private HopperConstants() {}
}
