package frc.robot.subsystems.intakerollers;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  public enum Goal {
    STOP,
    INTAKE,
    EJECT
  }

  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
  private Goal goal = Goal.STOP;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);

    switch (goal) {
      case STOP -> io.stop();
      case INTAKE -> io.setVoltage(IntakeRollersConstants.INTAKE_VOLTS);
      case EJECT -> io.setVoltage(IntakeRollersConstants.EJECT_VOLTS);
    }

    Logger.recordOutput("IntakeRollers/Goal", goal);
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
}
