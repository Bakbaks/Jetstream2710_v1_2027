package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  public enum Speed {
    STOP(0),
    INTAKE(IntakeConstants.kRollerPercent),
    OUTAKE(-IntakeConstants.kRollerPercent);
    final double percent;

    Speed(double percent) {
      this.percent = percent;
    }
  }

  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
  private Speed goal = Speed.STOP;
  private double percent;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);
    io.setVoltage(percent * 12.0);
    Logger.recordOutput("Intake/Rollers/Goal", goal);
  }

  public void setIntakeSpeed(Speed speed) {
    goal = speed;
    percent = speed.percent;
  }

  public void setRollerPercentOutput(double output) {
    goal = output == 0 ? Speed.STOP : goal;
    percent = output;
  }
}
