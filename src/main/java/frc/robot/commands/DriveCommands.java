package frc.robot.commands;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.util.DriveInput;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class DriveCommands {
  private static final double HIGHWAY_MAX_X_METERS = 3.5;
  private static final double HIGHWAY_CENTER_Y_METERS = 4.034;
  private static final double HIGHWAY_MAX_START_DISTANCE_METERS = 2.0;
  private static final double HIGHWAY_DRIVER_OVERRIDE_THRESHOLD = 0.15;
  private static final double HIGHWAY_ENTRY_SPEED_METERS_PER_SECOND = 1.5;

  private DriveCommands() {}

  public static Command joystickDrive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation) {
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)-1.0;
    double maxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);
    var request =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    return drivetrain.applyRequest(
        () ->
            request
                .withVelocityX(-DriveInput.exponential(translationX.getAsDouble(), 2) * maxSpeed)
                .withVelocityY(-DriveInput.exponential(translationY.getAsDouble(), 2) * maxSpeed)
                .withRotationalRate(
                    -DriveInput.exponential(rotation.getAsDouble(), 2) * maxAngularRate));
  }

  public static Command joystickDriveAtHeading(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      Supplier<Rotation2d> targetHeading) {
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) - 1.0;
    double maxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);
    var request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(10.0, 0.0, 0.0)
            .withMaxAbsRotationalRate(maxAngularRate)
            .withTargetDirectionPerspective(
                SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance);
    return drivetrain.applyRequest(
        () ->
            request
                .withVelocityX(-DriveInput.exponential(translationX.getAsDouble(), 2) * maxSpeed)
                .withVelocityY(-DriveInput.exponential(translationY.getAsDouble(), 2) * maxSpeed)
                .withTargetDirection(targetHeading.get()));
  }

  public static Command highwayAssist(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverRotation) {
    var coastRequest = new SwerveRequest.Idle();
    return Commands.defer(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();
          if (robotPose.getX() >= HIGHWAY_MAX_X_METERS) {
            Logger.recordOutput("Highway/Eligible", false);
            Logger.recordOutput("Highway/RejectionReason", "OutsideXRegion");
            return Commands.none();
          }

          String pathName =
              robotPose.getY() < HIGHWAY_CENTER_Y_METERS
                  ? "RIGHTHIGHWAY"
                  : "LEFTHIGHWAY";
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            PathPlannerPath fieldPath = AutoBuilder.shouldFlip() ? path.flipPath() : path;
            var startingPose = fieldPath.getStartingHolonomicPose();
            if (startingPose.isEmpty()) {
              Logger.recordOutput("Highway/Eligible", false);
              Logger.recordOutput("Highway/RejectionReason", "MissingStartingPose");
              return Commands.none();
            }

            double distanceToStart =
                robotPose.getTranslation().getDistance(startingPose.get().getTranslation());
            Logger.recordOutput("Highway/SelectedPath", pathName);
            Logger.recordOutput("Highway/DistanceToStartMeters", distanceToStart);
            if (distanceToStart > HIGHWAY_MAX_START_DISTANCE_METERS) {
              Logger.recordOutput("Highway/Eligible", false);
              Logger.recordOutput("Highway/RejectionReason", "TooFarFromStart");
              return Commands.none();
            }

            Logger.recordOutput("Highway/Eligible", true);
            Logger.recordOutput("Highway/RejectionReason", "None");
            return Commands.sequence(
                    AutoBuilder.pathfindToPose(
                        startingPose.get(),
                        path.getGlobalConstraints(),
                        MetersPerSecond.of(HIGHWAY_ENTRY_SPEED_METERS_PER_SECOND)),
                    AutoBuilder.followPath(path))
                .until(
                    () ->
                        hasDriverInput(driverX, driverY, driverRotation))
                .beforeStarting(() -> Logger.recordOutput("Highway/Running", true))
                .finallyDo(
                    () -> {
                      drivetrain.setControl(coastRequest);
                      Logger.recordOutput("Highway/Running", false);
                    });
          } catch (Exception exception) {
            Logger.recordOutput("Highway/Eligible", false);
            Logger.recordOutput("Highway/RejectionReason", "PathLoadFailed");
            return Commands.none();
          }
        },
        Set.of(drivetrain));
  }

  private static boolean hasDriverInput(
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverRotation) {
    return Math.hypot(driverX.getAsDouble(), driverY.getAsDouble())
            > HIGHWAY_DRIVER_OVERRIDE_THRESHOLD
        || Math.abs(driverRotation.getAsDouble()) > HIGHWAY_DRIVER_OVERRIDE_THRESHOLD;
  }
}
