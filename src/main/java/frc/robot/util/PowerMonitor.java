package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/** Basic robot power telemetry without power-budget or battery-model behavior. */
public final class PowerMonitor implements AutoCloseable {
  private final PowerDistribution pdh;

  public PowerMonitor() {
    PowerDistribution detected;
    try {
      detected = new PowerDistribution();
    } catch (RuntimeException exception) {
      detected = null;
    }
    pdh = detected;
  }

  public void periodic() {
    Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Power/BrownedOut", RobotController.isBrownedOut());
    if (pdh != null) {
      Logger.recordOutput("Power/TotalCurrentAmps", pdh.getTotalCurrent());
      Logger.recordOutput("Power/TotalPowerWatts", pdh.getTotalPower());
      Logger.recordOutput("Power/TemperatureCelsius", pdh.getTemperature());
    }
  }

  @Override
  public void close() {
    if (pdh != null) pdh.close();
  }
}
