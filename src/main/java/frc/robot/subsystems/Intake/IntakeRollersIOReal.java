package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;

public class IntakeRollersIOReal implements IntakeRollersIO {
  private final TalonFX[] motors = {
    new TalonFX(Ports.kIntakeRollers), new TalonFX(Ports.kIntakeRollers2)
  };
  private final VoltageOut request = new VoltageOut(0);

  public IntakeRollersIOReal() {
    InvertedValue[] inv = {
      InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive
    };
    for (int x = 0; x < 2; x++)
      motors[x]
          .getConfigurator()
          .apply(
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(inv[x])
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(
                              Amps.of(IntakeConstants.kRollerStatorCurrentLimit))
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(
                              Amps.of(IntakeConstants.kRollerSupplyCurrentLimit))
                          .withSupplyCurrentLimitEnable(true)));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs i) {
    for (int x = 0; x < 2; x++) {
      i.connected[x] = motors[x].isConnected();
      i.velocityRPM[x] = motors[x].getVelocity().getValue().in(RPM);
      i.appliedVolts[x] = motors[x].getMotorVoltage().getValue().in(Volts);
      i.supplyCurrentAmps[x] = motors[x].getSupplyCurrent().getValue().in(Amps);
      i.statorCurrentAmps[x] = motors[x].getStatorCurrent().getValue().in(Amps);
      i.temperatureCelsius[x] = motors[x].getDeviceTemp().getValue().in(Celsius);
    }
  }

  @Override
  public void setVoltage(double volts) {
    for (TalonFX m : motors) m.setControl(request.withOutput(Volts.of(volts)));
  }
}
