package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class SpinFloor extends Command {
  private final Hopper m_hopper;

  // private final Hopper m_hopper;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public SpinFloor(Hopper hopper) {
    m_hopper = hopper;
    // m_hopper = hopper;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Robot Pose to Goal distance

    // m_intakeRollers.setIntakeSpeed(Speed.INTAKE);
    m_hopper.setFloorRPM(500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intakeRollers.setIntakeSpeed(Speed.STOP);
    m_hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
