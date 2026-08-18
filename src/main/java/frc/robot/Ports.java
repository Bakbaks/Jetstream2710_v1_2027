package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  public static final CANBus RIO_CAN_BUS = new CANBus("rio");

  private Ports() {}
}
