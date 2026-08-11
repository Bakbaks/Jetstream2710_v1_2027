package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum Goal {
    STOP,
    VELOCITY,
    OPEN_LOOP
  }

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private Goal goal = Goal.STOP;
  private double floorRPM = HopperConstants.kFloorRPM;
  private double feederRPM = HopperConstants.kFeederRPM;
  private double floorPercent;
  private double feederPercent;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    switch (goal) {
      case VELOCITY -> io.setVelocityRPM(floorRPM, feederRPM);
      case OPEN_LOOP -> io.setVoltage(floorPercent * 12.0, feederPercent * 12.0);
      default -> io.stop();
    }
    Logger.recordOutput("Hopper/Goal", goal);
    Logger.recordOutput("Hopper/FloorGoalRPM", floorRPM);
    Logger.recordOutput("Hopper/FeederGoalRPM", feederRPM);
  }

  public void setPercentOutputs(double floor, double feeder) {
    goal = Goal.OPEN_LOOP;
    floorPercent = floor;
    feederPercent = feeder;
  }

  public void stop() {
    goal = Goal.STOP;
  }

  public void setFeederRPM() {
    setFeederRPM(HopperConstants.kFeederRPM);
  }

  public void setFeederRPM(double rpm) {
    goal = Goal.VELOCITY;
    feederRPM = rpm;
  }

  public void setFloorRPM() {
    setFloorRPM(HopperConstants.kFloorRPM);
  }

  public void setFloorRPM(double rpm) {
    goal = Goal.VELOCITY;
    floorRPM = rpm;
  }
}
