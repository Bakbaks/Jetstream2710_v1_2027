package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.junit.jupiter.api.Test;

class RobotContainerSimulationTest {
  @Test
  void constructsWithoutRobotHardware() {
    assertTrue(HAL.initialize(500, 0));
    assertDoesNotThrow(RobotContainer::new);
  }
}
