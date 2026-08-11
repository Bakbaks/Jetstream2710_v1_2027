package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants.*;
import frc.robot.Ports;

public class HopperIOReal implements HopperIO {
  private final TalonFX[] motors = {
    new TalonFX(Ports.kFloorL, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kFeederL, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kFloorR, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kFeederR, Ports.kRoboRioCANBus)
  };
  private final VelocityVoltage velocity = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltage = new VoltageOut(0);

  public HopperIOReal() {
    InvertedValue[] inversions = {
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.Clockwise_Positive
    };
    for (int i = 0; i < 4; i++)
      motors[i]
          .getConfigurator()
          .apply(
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(inversions[i])
                          .withNeutralMode(NeutralModeValue.Coast))
                  .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(-12)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(
                              Amps.of(HopperConstants.kHopperStatorCurrentLimit))
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(
                              Amps.of(HopperConstants.kHopperSupplyCurrentLimit))
                          .withSupplyCurrentLimitEnable(true))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(HopperConstants.KHopperP)
                          .withKI(HopperConstants.KHopperI)
                          .withKD(HopperConstants.KHopperD)
                          .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    for (int i = 0; i < 4; i++) {
      inputs.connected[i] = motors[i].isConnected();
      inputs.velocityRPM[i] = motors[i].getVelocity().getValue().in(RPM);
      inputs.appliedVolts[i] = motors[i].getMotorVoltage().getValue().in(Volts);
      inputs.supplyCurrentAmps[i] = motors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.statorCurrentAmps[i] = motors[i].getStatorCurrent().getValue().in(Amps);
      inputs.temperatureCelsius[i] = motors[i].getDeviceTemp().getValue().in(Celsius);
    }
  }

  @Override
  public void setVelocityRPM(double floor, double feeder) {
    motors[0].setControl(velocity.withVelocity(RPM.of(floor)));
    motors[2].setControl(velocity.withVelocity(RPM.of(floor)));
    motors[1].setControl(velocity.withVelocity(RPM.of(feeder)));
    motors[3].setControl(velocity.withVelocity(RPM.of(feeder)));
  }

  @Override
  public void setVoltage(double floor, double feeder) {
    motors[0].setControl(voltage.withOutput(Volts.of(floor)));
    motors[2].setControl(voltage.withOutput(Volts.of(floor)));
    motors[1].setControl(voltage.withOutput(Volts.of(feeder)));
    motors[3].setControl(voltage.withOutput(Volts.of(feeder)));
  }
}
