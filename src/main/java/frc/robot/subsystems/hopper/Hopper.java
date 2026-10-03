package frc.robot.subsystems.hopper;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum Goal {
    STOP,
    HOLD,
    FEED,
    REVERSE
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
      case STOP -> io.stop();
      case HOLD -> {io.setFeederVoltage(0); io.setFloorVoltage(0);}
      case FEED -> {io.setFeederVoltage(12.0); io.setFloorVoltage(10.0);}
          //io.setVelocityRPM(HopperConstants.FEED_FLOOR_RPM, HopperConstants.FEED_FEEDER_RPM);
          
      case REVERSE -> {io.setFeederVoltage(-6.0); io.setFloorVoltage(-8.0);}
    }
    Logger.recordOutput("Hopper/Goal", goal);
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
