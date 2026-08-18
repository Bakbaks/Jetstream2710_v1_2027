package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  public static final CANBus RIO_CAN_BUS = CANBus.systemcore(1);

  private Ports() {}
}
