package frc.robot.subsystems.intake;

import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum Goal {
    STOW,
    DEPLOY,
    INTAKE,
    EJECT,
    STOP
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Goal goal = Goal.STOW;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    switch (goal) {
      case STOW -> {
        io.setExtensionPositionInches(IntakeConstants.RETRACTED_INCHES);
        io.setRollerVoltage(0.0);
      }
      case DEPLOY -> {
        io.setExtensionPositionInches(IntakeConstants.EXTENDED_INCHES);
        io.setRollerVoltage(0.0);
      }
      case INTAKE -> {
        io.setExtensionPositionInches(IntakeConstants.EXTENDED_INCHES);
        io.setRollerVoltage(IntakeConstants.ROLLER_INTAKE_VOLTS);
      }
      case EJECT -> {
        io.setExtensionPositionInches(IntakeConstants.EXTENDED_INCHES);
        io.setRollerVoltage(IntakeConstants.ROLLER_EJECT_VOLTS);
      }
      case STOP -> io.setRollerVoltage(0.0);
    }
    Logger.recordOutput("Intake/Goal", goal);
    Logger.recordOutput("Intake/AtExtensionGoal", isAtExtensionGoal());
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public boolean isAtExtensionGoal() {
    double target =
        switch (goal) {
          case DEPLOY, INTAKE, EJECT -> IntakeConstants.EXTENDED_INCHES;
          case STOW -> IntakeConstants.RETRACTED_INCHES;
          case STOP -> inputs.extensionPositionInches;
        };
    return Math.abs(inputs.extensionPositionInches - target)
        <= IntakeConstants.POSITION_TOLERANCE_INCHES;
  }

  public void zeroExtensionEncoder() {
    io.setExtensionEncoderZero();
  }
}
