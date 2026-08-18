package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Ports;

public class ShooterIOReal implements ShooterIO {
  private static final int[] MOTOR_IDS = {16, 14, 17, 15};
  private final TalonFX[] motors = new TalonFX[4];
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOReal() {
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonFX(MOTOR_IDS[i], Ports.RIO_CAN_BUS);
      configureMotor(
          motors[i],
          i < 2 ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    }
  }

  private static void configureMotor(TalonFX motor, InvertedValue inversion) {
    var slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.VELOCITY_KP)
            .withKI(ShooterConstants.VELOCITY_KI)
            .withKD(ShooterConstants.VELOCITY_KD)
            .withKV(12.0 / RPM.of(ShooterConstants.FREE_SPEED_RPM).in(RotationsPerSecond));
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(-12.0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(ShooterConstants.STATOR_CURRENT_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(slot0);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double velocityTotal = 0.0;
    for (int i = 0; i < motors.length; i++) {
      inputs.connected[i] = motors[i].isConnected();
      inputs.velocityRPM[i] = motors[i].getVelocity().getValue().in(RPM);
      inputs.appliedVolts[i] = motors[i].getMotorVoltage().getValue().in(Volts);
      inputs.supplyVolts[i] = motors[i].getSupplyVoltage().getValue().in(Volts);
      inputs.supplyCurrentAmps[i] = motors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.statorCurrentAmps[i] = motors[i].getStatorCurrent().getValue().in(Amps);
      inputs.temperatureCelsius[i] = motors[i].getDeviceTemp().getValue().in(Celsius);
      velocityTotal += inputs.velocityRPM[i];
    }
    inputs.averageVelocityRPM = velocityTotal / motors.length;
  }

  @Override
  public void setVelocityRPM(double rpm) {
    for (TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
  }

  @Override
  public void setVoltage(double volts) {
    for (TalonFX motor : motors) {
      motor.setControl(voltageRequest.withOutput(Volts.of(volts)));
    }
  }
}
