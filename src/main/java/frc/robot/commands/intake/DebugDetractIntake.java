package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeExtendo;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class DebugDetractIntake extends Command {
  private final IntakeExtendo m_intakeExtendo;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public DebugDetractIntake(IntakeExtendo intakeExtendo) {
    m_intakeExtendo = intakeExtendo;
    addRequirements(m_intakeExtendo);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_intakeExtendo.setExtendoPercentOutput(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeExtendo.setExtendoPercentOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
