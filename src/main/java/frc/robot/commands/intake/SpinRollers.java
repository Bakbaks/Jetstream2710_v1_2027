package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeRollers.Speed;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class SpinRollers extends Command {
  private final IntakeRollers m_intakeRollers;

  // private final Hopper m_hopper;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public SpinRollers(IntakeRollers intakeRollers) {
    m_intakeRollers = intakeRollers;
    // m_hopper = hopper;
    addRequirements(m_intakeRollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Robot Pose to Goal distance

    m_intakeRollers.setIntakeSpeed(Speed.INTAKE);
    // m_hopper.setFloorRPM(500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeRollers.setIntakeSpeed(Speed.STOP);
    // m_hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
