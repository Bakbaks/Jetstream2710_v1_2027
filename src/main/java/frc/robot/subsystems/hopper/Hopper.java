package frc.robot.subsystems.hopper;

import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum Goal {
    STOP,
    HOLD,
    FEED,
    REVERSE,
    UNJAM
  }

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private Goal goal = Goal.STOP;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    switch (goal) {
      case STOP, HOLD -> io.stop();
      case FEED ->
          io.setVelocityRPM(HopperConstants.FEED_FLOOR_RPM, HopperConstants.FEED_FEEDER_RPM);
      case REVERSE -> io.setVelocityRPM(HopperConstants.REVERSE_RPM, HopperConstants.REVERSE_RPM);
      case UNJAM -> io.setVelocityRPM(HopperConstants.UNJAM_RPM, 0.0);
    }
    Logger.recordOutput("Hopper/Goal", goal);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }
}
