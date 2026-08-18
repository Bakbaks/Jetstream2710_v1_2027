package frc.robot.subsystems.shooter;

import org.wpilib.math.util.MathUtil;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum Goal {
    STOP,
    IDLE,
    SHOOT,
    REVERSE,
    CHARACTERIZE
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private Goal goal = Goal.STOP;
  private double requestedShotRPM = ShooterConstants.DEFAULT_SHOT_RPM;
  private double characterizationVolts;
  private double setpointRPM;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    switch (goal) {
      case STOP -> {
        io.stop();
        setpointRPM = inputs.averageVelocityRPM;
      }
      case IDLE -> runVelocity(ShooterConstants.IDLE_RPM);
      case SHOOT -> runVelocity(requestedShotRPM);
      case REVERSE -> runVelocity(ShooterConstants.REVERSE_RPM);
      case CHARACTERIZE -> io.setVoltage(characterizationVolts);
    }

    Logger.recordOutput("Shooter/Goal", goal);
    Logger.recordOutput("Shooter/RequestedRPM", requestedShotRPM);
    Logger.recordOutput("Shooter/SetpointRPM", setpointRPM);
    Logger.recordOutput("Shooter/Ready", isReady());
  }

  private void runVelocity(double targetRPM) {
    double maxStep =
        ShooterConstants.MAX_ACCELERATION_RPM_PER_SECOND * Constants.LOOP_PERIOD_SECONDS;
    setpointRPM += Math.clamp(targetRPM - setpointRPM, -maxStep, maxStep);
    io.setVelocityRPM(setpointRPM);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public void setShotRPM(double rpm) {
    requestedShotRPM = rpm;
  }

  public void setCharacterizationVolts(double volts) {
    characterizationVolts = volts;
    goal = Goal.CHARACTERIZE;
  }

  public boolean isReady() {
    if (goal != Goal.SHOOT || requestedShotRPM < ShooterConstants.MIN_READY_TARGET_RPM) {
      return false;
    }
    for (int i = 0; i < inputs.velocityRPM.length; i++) {
      if (!inputs.connected[i]
          || Math.abs(inputs.velocityRPM[i] - requestedShotRPM)
              > ShooterConstants.READY_TOLERANCE_RPM) {
        return false;
      }
    }
    return true;
  }

  public double getAverageVelocityRPM() {
    return inputs.averageVelocityRPM;
  }

  public double[] getMotorVelocitiesRPM() {
    return Arrays.copyOf(inputs.velocityRPM, inputs.velocityRPM.length);
  }
}
