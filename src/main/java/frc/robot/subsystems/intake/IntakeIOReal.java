package frc.robot.subsystems.intake;

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

public class IntakeIOReal implements IntakeIO {
  private static final int EXTENSION_MOTOR_ID = 12;
  private static final int ROLLER_LEFT_ID = 13;
  private static final int ROLLER_RIGHT_ID = 20;
  private final TalonFX extensionMotor = new TalonFX(EXTENSION_MOTOR_ID, Ports.RIO_CAN_BUS);
  private final TalonFX[] rollerMotors = {
    new TalonFX(ROLLER_LEFT_ID, Ports.RIO_CAN_BUS), new TalonFX(ROLLER_RIGHT_ID, Ports.RIO_CAN_BUS)
  };
  private final MotionMagicVoltage extensionRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private final VoltageOut rollerRequest = new VoltageOut(0.0);

  public IntakeIOReal() {
    configureExtensionMotor();
    configureRollerMotor(rollerMotors[0], InvertedValue.CounterClockwise_Positive);
    configureRollerMotor(rollerMotors[1], InvertedValue.Clockwise_Positive);
  }

  private void configureExtensionMotor() {
    var maxMechanismSpeed =
        RPM.of(IntakeConstants.FREE_SPEED_RPM).div(IntakeConstants.MOTOR_TO_PINION_REDUCTION);
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.EXTENSION_STATOR_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.EXTENSION_SUPPLY_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(IntakeConstants.MOTOR_TO_PINION_REDUCTION))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxMechanismSpeed)
                    .withMotionMagicAcceleration(maxMechanismSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeConstants.EXTENSION_KP)
                    .withKI(IntakeConstants.EXTENSION_KI)
                    .withKD(IntakeConstants.EXTENSION_KD)
                    .withKV(12.0 / maxMechanismSpeed.in(RotationsPerSecond)));
    extensionMotor.getConfigurator().apply(config);
  }

  private static void configureRollerMotor(TalonFX motor, InvertedValue inversion) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.ROLLER_STATOR_LIMIT_AMPS))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.ROLLER_SUPPLY_LIMIT_AMPS))
                    .withSupplyCurrentLimitEnable(true));
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double pinionRotations = extensionMotor.getPosition().getValue().in(Rotations);
    inputs.extensionConnected = extensionMotor.isConnected();
    inputs.extensionPositionInches = pinionRotations * IntakeConstants.INCHES_PER_PINION_ROTATION;
    inputs.extensionVelocityRPM = extensionMotor.getVelocity().getValue().in(RPM);
    inputs.extensionAppliedVolts = extensionMotor.getMotorVoltage().getValue().in(Volts);
    inputs.extensionSupplyCurrentAmps = extensionMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.extensionStatorCurrentAmps = extensionMotor.getStatorCurrent().getValue().in(Amps);
    inputs.extensionTemperatureCelsius = extensionMotor.getDeviceTemp().getValue().in(Celsius);
    for (int i = 0; i < rollerMotors.length; i++) {
      inputs.rollerConnected[i] = rollerMotors[i].isConnected();
      inputs.rollerVelocityRPM[i] = rollerMotors[i].getVelocity().getValue().in(RPM);
      inputs.rollerAppliedVolts[i] = rollerMotors[i].getMotorVoltage().getValue().in(Volts);
      inputs.rollerSupplyCurrentAmps[i] = rollerMotors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.rollerStatorCurrentAmps[i] = rollerMotors[i].getStatorCurrent().getValue().in(Amps);
      inputs.rollerTemperatureCelsius[i] = rollerMotors[i].getDeviceTemp().getValue().in(Celsius);
    }
  }

  @Override
  public void setExtensionPositionInches(double positionInches) {
    extensionMotor.setControl(
        extensionRequest.withPosition(
            Rotations.of(positionInches / IntakeConstants.INCHES_PER_PINION_ROTATION)));
  }

  @Override
  public void setRollerVoltage(double volts) {
    for (TalonFX motor : rollerMotors) {
      motor.setControl(rollerRequest.withOutput(Volts.of(volts)));
    }
  }

  @Override
  public void setExtensionEncoderZero() {
    extensionMotor.setPosition(0.0);
  }
}
