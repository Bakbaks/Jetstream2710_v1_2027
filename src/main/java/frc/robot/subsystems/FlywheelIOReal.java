package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX[] motors = {
    new TalonFX(Ports.kTopLeftShooter, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kBottomLeftShooter, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kTopRightShooter, Ports.kRoboRioCANBus),
    new TalonFX(Ports.kBottomRightShooter, Ports.kRoboRioCANBus)
  };
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Slot0Configs slot0 = new Slot0Configs();
  private boolean closedLoop;

  public FlywheelIOReal() {
    for (int i = 0; i < motors.length; i++)
      configure(
          motors[i],
          i < 2 ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
  }

  private void configure(TalonFX motor, InvertedValue inverted) {
    slot0
        .withKP(FlywheelConstants.KFlywheelP)
        .withKI(FlywheelConstants.KFlywheelI)
        .withKD(FlywheelConstants.KFlywheelD)
        .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond));
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(inverted)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(-12)))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(
                            Amps.of(FlywheelConstants.FlywheelStatorCurrentLimit))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(
                            Amps.of(FlywheelConstants.FlywheelSupplyCurrentLimit))
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(slot0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double total = 0.0;
    for (int i = 0; i < motors.length; i++) {
      inputs.connected[i] = motors[i].isConnected();
      inputs.velocityRPM[i] = motors[i].getVelocity().getValue().in(RPM);
      inputs.appliedVolts[i] = motors[i].getMotorVoltage().getValue().in(Volts);
      inputs.supplyCurrentAmps[i] = motors[i].getSupplyCurrent().getValue().in(Amps);
      inputs.statorCurrentAmps[i] = motors[i].getStatorCurrent().getValue().in(Amps);
      inputs.temperatureCelsius[i] = motors[i].getDeviceTemp().getValue().in(Celsius);
      total += inputs.velocityRPM[i];
    }
    inputs.averageVelocityRPM = total / motors.length;
    inputs.closedLoop = closedLoop;
  }

  @Override
  public void setVelocityRPM(double rpm) {
    for (TalonFX motor : motors) motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    closedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    for (TalonFX motor : motors) motor.setControl(voltageRequest.withOutput(Volts.of(volts)));
    closedLoop = false;
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA) {
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    for (TalonFX motor : motors) motor.getConfigurator().apply(slot0);
  }

  @Override
  public double getFeedforwardKS() {
    return slot0.kS;
  }

  @Override
  public double getFeedforwardKV() {
    return slot0.kV;
  }

  @Override
  public double getFeedforwardKA() {
    return slot0.kA;
  }
}
