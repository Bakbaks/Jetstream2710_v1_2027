package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Read-only view of the robot state consumed by planners and coordination logic. */
public final class RobotState {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisVelocities> fieldRelativeSpeedsSupplier;

  public RobotState(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisVelocities> fieldRelativeSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
  }

  public Pose2d getPose() {
    return poseSupplier.get();
  }

  public ChassisVelocities getFieldRelativeSpeeds() {
    return fieldRelativeSpeedsSupplier.get();
  }

  public void periodic() {
    Logger.recordOutput("RobotState/Pose", getPose());
    Logger.recordOutput("RobotState/FieldRelativeSpeeds", getFieldRelativeSpeeds());
  }
}
