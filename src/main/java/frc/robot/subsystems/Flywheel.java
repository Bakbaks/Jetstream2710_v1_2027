package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  public enum Goal {
    STOP,
    VELOCITY,
    OPEN_LOOP
  }

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private Goal goal = Goal.STOP;
  private double requestedRPM;
  private double openLoopPercent;
  private double setpointRPM;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    if (goal == Goal.VELOCITY) {
      double maxStep = FlywheelConstants.kMaxAccelerationRPMPerSecond * 0.02;
      setpointRPM += MathUtil.clamp(requestedRPM - setpointRPM, -maxStep, maxStep);
      io.setVelocityRPM(setpointRPM);
    } else if (goal == Goal.OPEN_LOOP) {
      io.setVoltage(openLoopPercent * 12.0);
      setpointRPM = inputs.averageVelocityRPM;
    } else {
      io.stop();
      setpointRPM = inputs.averageVelocityRPM;
    }
    Logger.recordOutput("Flywheel/Goal", goal);
    Logger.recordOutput("Flywheel/RequestedRPM", requestedRPM);
    Logger.recordOutput("Flywheel/SetpointRPM", setpointRPM);
    Logger.recordOutput("Flywheel/AtGoal", isVelocityWithinTolerance());
  }

  public void setRPM(double rpm) {
    goal = Goal.VELOCITY;
    requestedRPM = rpm;
  }

  public void setPercentOutput(double percent) {
    goal = Goal.OPEN_LOOP;
    openLoopPercent = percent;
  }

  public void stop() {
    goal = Goal.STOP;
  }

  public void setFeedforwardKS(double value) {
    io.setFeedforward(value, getFeedforwardKV(), getFeedforwardKA());
  }

  public void setFeedforwardKV(double value) {
    io.setFeedforward(getFeedforwardKS(), value, getFeedforwardKA());
  }

  public void setFeedforwardKA(double value) {
    io.setFeedforward(getFeedforwardKS(), getFeedforwardKV(), value);
  }

  public double getFeedforwardKS() {
    return io.getFeedforwardKS();
  }

  public double getFeedforwardKV() {
    return io.getFeedforwardKV();
  }

  public double getFeedforwardKA() {
    return io.getFeedforwardKA();
  }

  public Command spinUpCommand(double rpm) {
    return runOnce(() -> setRPM(rpm)).andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  public double getRequestedRPM() {
    return requestedRPM;
  }

  public double getFlywheelRPM() {
    return inputs.averageVelocityRPM;
  }

  public boolean isVelocityWithinTolerance() {
    return requestedRPM >= FlywheelConstants.kMinTargetRPM
        && inputs.closedLoop
        && Math.abs(inputs.averageVelocityRPM - requestedRPM)
            <= FlywheelConstants.kVelocityTolerance;
  }
}
