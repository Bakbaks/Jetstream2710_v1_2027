package frc.robot.power;

import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;
import org.littletonrobotics.junction.Logger;

public final class PowerMonitor implements AutoCloseable {
  private final PowerDistribution powerDistribution;

  public PowerMonitor() {
    PowerDistribution detected;
    try {
      detected = new PowerDistribution(PowerDistribution.kDefaultModule);
    } catch (RuntimeException exception) {
      detected = null;
    }
    powerDistribution = detected;
  }

  public void periodic() {
    Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Power/BrownedOut", RobotController.isBrownedOut());
    if (powerDistribution != null) {
      Logger.recordOutput("Power/TotalCurrentAmps", powerDistribution.getTotalCurrent());
      Logger.recordOutput("Power/TotalPowerWatts", powerDistribution.getTotalPower());
      Logger.recordOutput("Power/TemperatureCelsius", powerDistribution.getTemperature());
    }
  }

  @Override
  public void close() {
    if (powerDistribution != null) {
      powerDistribution.close();
    }
  }
}
