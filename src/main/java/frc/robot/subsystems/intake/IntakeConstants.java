package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final double ROLLER_INTAKE_VOLTS = 9.0;
  public static final double ROLLER_EJECT_VOLTS = -9.0;
  public static final double RETRACTED_INCHES = 0.0;
  public static final double EXTENDED_INCHES = 6.0;
  public static final double POSITION_TOLERANCE_INCHES = 0.25;
  public static final double MOTOR_TO_PINION_REDUCTION = 5.0;
  public static final double INCHES_PER_PINION_ROTATION = Math.PI / 3.0;
  public static final double EXTENSION_STATOR_LIMIT_AMPS = 60.0;
  public static final double EXTENSION_SUPPLY_LIMIT_AMPS = 20.0;
  public static final double ROLLER_STATOR_LIMIT_AMPS = 100.0;
  public static final double ROLLER_SUPPLY_LIMIT_AMPS = 80.0;

  // TODO: Unvalidated values retained from the existing robot.
  public static final double EXTENSION_KP = 0.5;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.0;
  public static final double FREE_SPEED_RPM = 6000.0;

  private IntakeConstants() {}
}
