package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants.*;
import frc.robot.Ports;

public class IntakeExtendoIOReal implements IntakeExtendoIO {
  private final TalonFX motor = new TalonFX(Ports.kIntakeExtendo);
  private final VoltageOut voltage = new VoltageOut(0);
  private final MotionMagicVoltage position = new MotionMagicVoltage(0).withSlot(0);

  public IntakeExtendoIOReal() {
    var max = KrakenX60.kFreeSpeed.div(IntakeConstants.kMotorToPinionReduction);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(IntakeConstants.kExtendoStatorCurrentLimit))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(IntakeConstants.kExtendoSupplyCurrentLimit))
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(IntakeConstants.kMotorToPinionReduction))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(max)
                        .withMotionMagicAcceleration(max.per(Second)))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(IntakeConstants.kExtendoIntakeP)
                        .withKI(IntakeConstants.kExtendoIntakeI)
                        .withKD(IntakeConstants.kExtendoIntakeD)
                        .withKV(12 / max.in(RotationsPerSecond))));
  }

  @Override
  public void updateInputs(IntakeExtendoIOInputs i) {
    i.connected = motor.isConnected();
    i.positionRotations = motor.getPosition().getValue().in(Rotations);
    i.positionInches = i.positionRotations * IntakeConstants.kInchesPerPinionRotation;
    i.velocityRPM = motor.getVelocity().getValue().in(RPM);
    i.appliedVolts = motor.getMotorVoltage().getValue().in(Volts);
    i.supplyCurrentAmps = motor.getSupplyCurrent().getValue().in(Amps);
    i.statorCurrentAmps = motor.getStatorCurrent().getValue().in(Amps);
    i.temperatureCelsius = motor.getDeviceTemp().getValue().in(Celsius);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltage.withOutput(Volts.of(volts)));
  }

  @Override
  public void setPositionInches(double inches) {
    motor.setControl(
        position.withPosition(Rotations.of(inches / IntakeConstants.kInchesPerPinionRotation)));
  }

  @Override
  public void setZero() {
    motor.setPosition(0);
  }
}
