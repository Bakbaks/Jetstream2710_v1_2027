package frc.robot.subsystems.intakeextension;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends SubsystemBase {
  public enum Goal {
    STOP,
    EXTEND,
    RETRACT
  }

  private final IntakeExtensionIO io;
  private final IntakeExtensionIOInputsAutoLogged inputs =
      new IntakeExtensionIOInputsAutoLogged();
  private Goal goal = Goal.STOP;

  public IntakeExtension(IntakeExtensionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeExtension", inputs);

    switch (goal) {
      case STOP -> io.stop();
      case EXTEND -> io.setVoltage(IntakeExtensionConstants.OPEN_LOOP_VOLTS);
      case RETRACT -> io.setVoltage(-IntakeExtensionConstants.OPEN_LOOP_VOLTS);
    }

    Logger.recordOutput("IntakeExtension/Goal", goal);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public Command goalCommand(Goal requestedGoal, Goal endGoal) {
    return startEnd(() -> setGoal(requestedGoal), () -> setGoal(endGoal));
  }

  public Command setGoalCommand(Goal requestedGoal) {
    return runOnce(() -> setGoal(requestedGoal));
  }

  public void zeroEncoder() {
    io.setEncoderZero();
  }
}
