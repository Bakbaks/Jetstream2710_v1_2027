package frc.robot.shooting;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Temporary distance table carried forward from v2. Values require on-field validation. */
public final class ShotTable {
  private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();

  public ShotTable() {
    rpmTable.put(1.7, 1250.0);
    rpmTable.put(3.0, 1450.0);
    rpmTable.put(3.6, 1590.0);
    rpmTable.put(4.3, 1670.0);
    timeOfFlightTable.put(1.7, 0.75);
    timeOfFlightTable.put(3.0, 0.905);
    timeOfFlightTable.put(3.6, 1.085);
    timeOfFlightTable.put(4.3, 1.085);
  }

  public double getRPM(double distanceMeters) {
    Double value = rpmTable.get(distanceMeters);
    return value != null ? value : ShooterConstants.DEFAULT_SHOT_RPM;
  }

  public double getTimeOfFlightSeconds(double distanceMeters) {
    Double value = timeOfFlightTable.get(distanceMeters);
    return value != null ? value : ShotConstants.DEFAULT_TIME_OF_FLIGHT_SECONDS;
  }
}
