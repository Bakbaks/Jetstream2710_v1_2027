package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  public static final CANBus FLOOREXTENDO_CAN_BUS = CANBus.systemcore(1);
  public static final CANBus INTAKE_CAN_BUS = CANBus.systemcore(2);
  public static final CANBus FEEDER_CAN_BUS = CANBus.systemcore(3);
  public static final CANBus FLYWHEEL_CAN_BUS = CANBus.systemcore(4);


  private Ports() {}
}
