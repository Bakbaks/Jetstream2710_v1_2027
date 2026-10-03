package frc.robot.subsystems.intakerollers;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Ports;

public class IntakeRollersIOReal implements IntakeRollersIO {
  private static final int LEFT_MOTOR_ID = 13;
  private static final int RIGHT_MOTOR_ID = 51;
  private final TalonFX[] motors = {
    new TalonFX(LEFT_MOTOR_ID, Ports.INTAKE_CAN_BUS),
    new TalonFX(RIGHT_MOTOR_ID, Ports.INTAKE_CAN_BUS)
  };
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeRollersIOReal() {
    configureMotor(motors[0], InvertedValue.CounterClockwise_Positive);
    configureMotor(motors[1], InvertedValue.Clockwise_Positive);
  }

  private static void configureMotor(TalonFX motor, InvertedValue inversion) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(
                        Amps.of(IntakeRollersConstants.STATOR_CURRENT_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(
                        Amps.of(IntakeRollersConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true));
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.connected[i] = motors[i].isConnected();
      inputs.velocityRPM[i] = motors[i].getVelocity().getValue().in(RPM);
      inputs.appliedVolts[i] = motors[i].getMotorVoltage().getValue().in(Volts);
      inputs.supplyCurrentAmps[i] = motors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.statorCurrentAmps[i] = motors[i].getStatorCurrent().getValue().in(Amps);
      inputs.temperatureCelsius[i] = motors[i].getDeviceTemp().getValue().in(Celsius);
    }
  }

  @Override
  public void setVoltage(double volts) {
    for (TalonFX motor : motors) {
      motor.setControl(voltageRequest.withOutput(Volts.of(volts)));
    }
  }
}
