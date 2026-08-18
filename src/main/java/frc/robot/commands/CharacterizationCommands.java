package frc.robot.commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public final class CharacterizationCommands {
  private CharacterizationCommands() {}

  public static Command shooterVoltage(Shooter shooter, DoubleSupplier volts) {
    return shooter.runEnd(
        () -> shooter.setCharacterizationVolts(volts.getAsDouble()),
        () -> shooter.setGoal(Shooter.Goal.STOP));
  }
}
