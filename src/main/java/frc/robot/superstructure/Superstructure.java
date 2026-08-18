package frc.robot.superstructure;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotConstants;
import frc.robot.shooting.ShotSolution;
import frc.robot.shooting.ShotVerifier;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum Goal {
    IDLE,
    INTAKE,
    PREP_SHOT,
    SHOOT,
    PASS,
    EJECT
  }

  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final ShotCalculator shotCalculator;
  private final ShotVerifier shotVerifier;
  private Goal goal = Goal.IDLE;
  private ShotSolution latestShotSolution;

  public Superstructure(
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShotCalculator shotCalculator,
      ShotVerifier shotVerifier) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.shotCalculator = shotCalculator;
    this.shotVerifier = shotVerifier;
  }

  @Override
  public void periodic() {
    switch (goal) {
      case IDLE -> {
        shooter.setGoal(Shooter.Goal.IDLE);
        hopper.setGoal(Hopper.Goal.HOLD);
        intake.setGoal(Intake.Goal.STOW);
      }
      case INTAKE -> {
        shooter.setGoal(Shooter.Goal.IDLE);
        hopper.setGoal(Hopper.Goal.HOLD);
        intake.setGoal(Intake.Goal.INTAKE);
      }
      case PREP_SHOT, SHOOT -> prepareShot(goal == Goal.SHOOT);
      case PASS -> {
        shooter.setShotRPM(ShooterConstants.PASS_RPM);
        shooter.setGoal(Shooter.Goal.SHOOT);
        hopper.setGoal(shooter.isReady() ? Hopper.Goal.FEED : Hopper.Goal.HOLD);
        intake.setGoal(Intake.Goal.STOW);
      }
      case EJECT -> {
        shooter.setGoal(Shooter.Goal.REVERSE);
        hopper.setGoal(Hopper.Goal.REVERSE);
        intake.setGoal(Intake.Goal.EJECT);
      }
    }
    Logger.recordOutput("Superstructure/Goal", goal);
  }

  private void prepareShot(boolean fireRequested) {
    latestShotSolution = shotCalculator.calculate(getAllianceTarget());
    shooter.setShotRPM(latestShotSolution.shooterRPM());
    shooter.setGoal(Shooter.Goal.SHOOT);
    hopper.setGoal(
        fireRequested && shotVerifier.canFire(latestShotSolution)
            ? Hopper.Goal.FEED
            : Hopper.Goal.HOLD);
    intake.setGoal(Intake.Goal.STOW);
  }

  private static Optional<org.wpilib.math.geometry.Translation2d> getAllianceTarget() {
    int targetTagId =
        MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED
            ? ShotConstants.RED_TARGET_TAG_ID
            : ShotConstants.BLUE_TARGET_TAG_ID;
    return ShotConstants.FIELD_LAYOUT
        .getTagPose(targetTagId)
        .map(tagPose -> tagPose.toPose2d().plus(ShotConstants.TAG_TO_TARGET).getTranslation());
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public Command goalCommand(Goal requestedGoal) {
    return startEnd(() -> setGoal(requestedGoal), () -> setGoal(Goal.IDLE));
  }

  public Command setGoalCommand(Goal requestedGoal) {
    return runOnce(() -> setGoal(requestedGoal));
  }

  public Optional<ShotSolution> getLatestShotSolution() {
    return Optional.ofNullable(latestShotSolution);
  }
}
