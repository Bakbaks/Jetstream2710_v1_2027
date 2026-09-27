package frc.robot.subsystems.intake;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum Goal {
    STOW,
    DEPLOY,
    INTAKE,
    EJECT,
    COMPRESSION,
    STOP
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Goal goal = Goal.STOW;
  private double holdPositionRotations;
  private boolean auxiliaryExtensionOverride;
  private boolean auxiliaryExtensionActive;
  private double auxiliaryExtensionVolts;
  private boolean auxiliaryRollersActive;
  private double auxiliaryRollerVolts;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    if (auxiliaryExtensionOverride) {
      io.setExtensionVoltage(auxiliaryExtensionVolts);
    } else {
      switch (goal) {
        case STOW -> {
          io.setExtensionPositionRotations(IntakeConstants.RETRACTED_ROTATIONS);
        }
        case DEPLOY -> {
          io.setExtensionPositionRotations(IntakeConstants.EXTENDED_ROTATIONS);
        }
        case INTAKE -> {
          io.setExtensionPositionRotations(IntakeConstants.EXTENDED_ROTATIONS);
        }
        case EJECT -> {
          io.setExtensionPositionRotations(IntakeConstants.EXTENDED_ROTATIONS);
        }
        case COMPRESSION -> {
          io.setExtensionPositionRotations(IntakeConstants.RETRACTED_ROTATIONS);
        }
        case STOP -> {
          io.setExtensionPositionRotations(holdPositionRotations);
        }
      }
    }
    if (auxiliaryRollersActive) {
      io.setRollerVoltage(auxiliaryRollerVolts);
    } else if (goal == Goal.INTAKE
        && inputs.extensionPositionRotations
            < IntakeConstants.INTAKE_ROLLER_DIRECTION_CHANGE_ROTATIONS || goal == Goal.COMPRESSION
        && inputs.extensionPositionRotations
            < IntakeConstants.INTAKE_ROLLER_DIRECTION_CHANGE_ROTATIONS) {
      io.setRollerVoltage(0.0);
    } else if (goal == Goal.INTAKE || goal == Goal.COMPRESSION) {
      io.setRollerVoltage(IntakeConstants.ROLLER_INTAKE_VOLTS);
    } else if (goal == Goal.EJECT) {
      io.setRollerVoltage(IntakeConstants.ROLLER_EJECT_VOLTS);
    } else {
      io.setRollerVoltage(0.0);
    }
    Logger.recordOutput("Intake/Goal", goal);
    Logger.recordOutput("Intake/AuxiliaryExtensionOverride", auxiliaryExtensionOverride);
    Logger.recordOutput("Intake/AuxiliaryExtensionActive", auxiliaryExtensionActive);
    Logger.recordOutput("Intake/AuxiliaryExtensionVolts", auxiliaryExtensionVolts);
    Logger.recordOutput("Intake/AtExtensionGoal", isAtExtensionGoal());
  }

  public void setGoal(Goal goal) {
    if (!auxiliaryExtensionActive) {
      auxiliaryExtensionOverride = false;
    }
    if (goal == Goal.STOP && this.goal != Goal.STOP) {
      holdPositionRotations = inputs.extensionPositionRotations;
    }
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public Command goalCommand(Goal requestedGoal, Goal endGoal) {
    return runEnd(() -> setGoal(requestedGoal), () -> setGoal(endGoal));
  }

  public Command setGoalCommand(Goal requestedGoal) {
    return runOnce(() -> setGoal(requestedGoal));
  }

  public Command auxiliaryOpenLoopCommand(double extensionVolts, double rollerVolts) {
    return Commands.startEnd(
        () -> {
          auxiliaryExtensionOverride = true;
          auxiliaryExtensionActive = true;
          auxiliaryExtensionVolts = extensionVolts;
          auxiliaryRollersActive = true;
          auxiliaryRollerVolts = rollerVolts;
        },
        () -> {
          auxiliaryExtensionOverride = true;
          auxiliaryExtensionActive = false;
          auxiliaryExtensionVolts = 0.0;
          auxiliaryRollersActive = false;
        });
  }

  public boolean isAtExtensionGoal() {
    double target =
        switch (goal) {
          case DEPLOY, INTAKE, EJECT -> IntakeConstants.EXTENDED_ROTATIONS;
          case STOW, COMPRESSION -> 0.0;
          case STOP -> inputs.extensionPositionRotations;
        };
    return Math.abs(inputs.extensionPositionRotations - target)
        <= IntakeConstants.POSITION_TOLERANCE_ROTATIONS;

  }

  public void zeroExtensionEncoder() {
    io.setExtensionEncoderZero();
  }
}
