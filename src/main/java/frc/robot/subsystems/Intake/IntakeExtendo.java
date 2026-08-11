package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeExtendo extends SubsystemBase {
  public enum Position {
    DEFAULT(0),
    INTERMEDIATE(3),
    EXTENDED(6);
    private final double inches;

    Position(double x) {
      inches = x;
    }

    public double inches() {
      return inches;
    }
  }

  public enum Goal {
    STOP,
    OPEN_LOOP,
    POSITION
  }

  private final IntakeExtendoIO io;
  private final IntakeExtendoIOInputsAutoLogged inputs = new IntakeExtendoIOInputsAutoLogged();
  private Goal goal = Goal.STOP;
  private double volts, targetInches;

  public IntakeExtendo(IntakeExtendoIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Extendo", inputs);
    if (goal == Goal.OPEN_LOOP) io.setVoltage(volts);
    else if (goal == Goal.POSITION) io.setPositionInches(targetInches);
    else io.setVoltage(0);
    Logger.recordOutput("Intake/Extendo/Goal", goal);
    Logger.recordOutput("Intake/Extendo/TargetInches", targetInches);
  }

  public void setExtendoPercentOutput(double percent) {
    goal = percent == 0 ? Goal.STOP : Goal.OPEN_LOOP;
    volts = percent * 12;
  }

  public void setExtendoInches(double inches) {
    goal = Goal.POSITION;
    targetInches = inches;
  }

  public void setExtendoPosition(Position position) {
    setExtendoInches(position.inches());
  }

  public void setExtendoZero() {
    io.setZero();
  }

  public double getExtendoPinionRotations() {
    return inputs.positionRotations;
  }

  public double getExtendoInches() {
    return inputs.positionInches;
  }

  public double getExtendoTargetInches() {
    return targetInches;
  }

  public Command jiggleToDefault() {
    double goalIn = Position.DEFAULT.inches(),
        freq = Math.max(0, IntakeConstants.kExtendoJiggleFrequencyHz),
        approach = Math.max(0, IntakeConstants.kExtendoJiggleStep.in(Inches)),
        minAmp = Math.max(0, IntakeConstants.kExtendoJiggleMinStep.in(Inches)),
        hold = Math.max(0, IntakeConstants.kExtendoJiggleHoldStep.in(Inches)),
        start = getExtendoInches(),
        total = Math.max(1e-6, Math.abs(goalIn - start));
    Timer timer = new Timer();
    return Commands.sequence(
        Commands.runOnce(timer::restart),
        Commands.run(
                () -> {
                  double remaining = Math.abs(goalIn - getExtendoInches());
                  double amp =
                      remaining <= IntakeConstants.kPositionTolerance.in(Inches)
                          ? hold
                          : Math.min(remaining, Math.max(minAmp, approach * remaining / total));
                  double jiggle =
                      freq > 0 ? amp * Math.abs(Math.sin(2 * Math.PI * freq * timer.get())) : 0;
                  setExtendoInches(
                      Math.max(
                          Position.DEFAULT.inches(),
                          Math.min(Position.EXTENDED.inches(), goalIn + jiggle)));
                })
            .finallyDo(interrupted -> timer.stop()));
  }
}
