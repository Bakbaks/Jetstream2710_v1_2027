package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Ports;

public class HopperIOReal implements HopperIO {
  private static final int FLOOR_LEFT = 0;
  private static final int FEEDER_LEFT = 1;
  private static final int FLOOR_RIGHT = 2;
  private static final int FEEDER_RIGHT = 3;
  private static final int FLOOR_LEFT_ID = 30;
  private static final int FLOOR_RIGHT_ID = 62;
  private static final int FEEDER_LEFT_ID = 52;
  private static final int FEEDER_RIGHT_ID = 51;

  private final TalonFX[] motors = {
    new TalonFX(FLOOR_LEFT_ID, Ports.RIO_CAN_BUS),
    new TalonFX(FEEDER_LEFT_ID, Ports.RIO_CAN_BUS),
    new TalonFX(FLOOR_RIGHT_ID, Ports.RIO_CAN_BUS),
    new TalonFX(FEEDER_RIGHT_ID, Ports.RIO_CAN_BUS)
  };
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

  public HopperIOReal() {
    InvertedValue[] inversions = {
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.Clockwise_Positive
    };
    for (int i = 0; i < motors.length; i++) {
      configureMotor(motors[i], inversions[i]);
    }
  }

  private static void configureMotor(TalonFX motor, InvertedValue inversion) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_CURRENT_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(HopperConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(HopperConstants.VELOCITY_KP)
                    .withKI(HopperConstants.VELOCITY_KI)
                    .withKD(HopperConstants.VELOCITY_KD)
                    .withKV(12.0 / RPM.of(HopperConstants.FREE_SPEED_RPM).in(RotationsPerSecond)));
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.connected[i] = motors[i].isConnected();
      inputs.velocityRPM[i] = motors[i].getVelocity().getValue().in(RPM);
      inputs.appliedVolts[i] = motors[i].getMotorVoltage().getValue().in(Volts);
      inputs.supplyVolts[i] = motors[i].getSupplyVoltage().getValue().in(Volts);
      inputs.supplyCurrentAmps[i] = motors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.statorCurrentAmps[i] = motors[i].getStatorCurrent().getValue().in(Amps);
      inputs.temperatureCelsius[i] = motors[i].getDeviceTemp().getValue().in(Celsius);
    }
  }

  @Override
  public void setVelocityRPM(double floorRPM, double feederRPM) {
    motors[FLOOR_LEFT].setControl(velocityRequest.withVelocity(RPM.of(floorRPM)));
    motors[FLOOR_RIGHT].setControl(velocityRequest.withVelocity(RPM.of(floorRPM)));
    motors[FEEDER_LEFT].setControl(velocityRequest.withVelocity(RPM.of(feederRPM)));
    motors[FEEDER_RIGHT].setControl(velocityRequest.withVelocity(RPM.of(feederRPM)));
  }
}
