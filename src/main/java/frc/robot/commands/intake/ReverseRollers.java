package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeRollers.Speed;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class ReverseRollers extends Command {
  private final IntakeRollers m_intakeRollers;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public ReverseRollers(IntakeRollers intakeRollers) {
    m_intakeRollers = intakeRollers;
    addRequirements(m_intakeRollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Robot Pose to Goal distance

    m_intakeRollers.setIntakeSpeed(Speed.OUTAKE);
    // m_hopper.setFloorRPM();
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
