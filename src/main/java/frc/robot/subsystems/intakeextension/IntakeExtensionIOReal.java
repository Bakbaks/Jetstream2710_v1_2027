package frc.robot.subsystems.intakeextension;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Ports;

public class IntakeExtensionIOReal implements IntakeExtensionIO {
  private static final int MOTOR_ID = 12;
  private final TalonFX motor = new TalonFX(MOTOR_ID, Ports.FLOOREXTENDO_CAN_BUS);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeExtensionIOReal() {
    var maxMechanismSpeed =
        RPM.of(IntakeExtensionConstants.FREE_SPEED_RPM)
            .div(IntakeExtensionConstants.MOTOR_TO_PINION_REDUCTION);
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(
                        Amps.of(IntakeExtensionConstants.STATOR_CURRENT_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(
                        Amps.of(IntakeExtensionConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(
                        IntakeExtensionConstants.MOTOR_TO_PINION_REDUCTION))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxMechanismSpeed)
                    .withMotionMagicAcceleration(maxMechanismSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeExtensionConstants.KP)
                    .withKI(IntakeExtensionConstants.KI)
                    .withKD(IntakeExtensionConstants.KD)
                    .withKV(12.0 / maxMechanismSpeed.in(RotationsPerSecond)));
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeExtensionIOInputs inputs) {
    double pinionRotations = motor.getPosition().getValue().in(Rotations);
    inputs.connected = motor.isConnected();
    inputs.positionInches =
        pinionRotations * IntakeExtensionConstants.INCHES_PER_PINION_ROTATION;
    inputs.positionRotations = pinionRotations;
    inputs.velocityRPM = motor.getVelocity().getValue().in(RPM);
    inputs.appliedVolts = motor.getMotorVoltage().getValue().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValue().in(Amps);
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValue().in(Amps);
    inputs.temperatureCelsius = motor.getDeviceTemp().getValue().in(Celsius);
  }

  @Override
  public void setPositionInches(double positionInches) {
    motor.setControl(
        positionRequest.withPosition(
            Rotations.of(
                positionInches / IntakeExtensionConstants.INCHES_PER_PINION_ROTATION)));
  }

  @Override
  public void setPositionRotations(double rotations) {
    motor.setControl(positionRequest.withPosition(Rotations.of(rotations)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(Volts.of(volts)));
  }

  @Override
  public void setEncoderZero() {
    motor.setPosition(0.0);
  }
}
