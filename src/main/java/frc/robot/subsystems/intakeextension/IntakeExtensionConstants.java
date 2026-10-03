package frc.robot.subsystems.intakeextension;

public final class IntakeExtensionConstants {
  public static final double OPEN_LOOP_VOLTS = 6.0;
  public static final double RETRACTED_INCHES = 0.0;
  public static final double EXTENDED_INCHES = 6.0;
  public static final double EXTENDED_ROTATIONS = 3.25;
  public static final double RETRACTED_ROTATIONS = -0.25;
  public static final double INTAKE_ROLLER_DIRECTION_CHANGE_ROTATIONS =
      (RETRACTED_ROTATIONS + EXTENDED_ROTATIONS) / 2.0;
  public static final double POSITION_TOLERANCE_INCHES = 0.25;
  public static final double POSITION_TOLERANCE_ROTATIONS = 0.25;
  public static final double MOTOR_TO_PINION_REDUCTION = 5.0;
  public static final double INCHES_PER_PINION_ROTATION = Math.PI / 3.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  // TODO: Unvalidated values retained from the existing robot.
  public static final double KP = 0.5;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double FREE_SPEED_RPM = 6000.0;

  private IntakeExtensionConstants() {}
}
