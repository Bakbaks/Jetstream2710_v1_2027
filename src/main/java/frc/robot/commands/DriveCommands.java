package frc.robot.commands;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import org.wpilib.command2.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.util.DriveInput;
import java.util.function.DoubleSupplier;

public final class DriveCommands {
  private DriveCommands() {}

  public static Command joystickDrive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation) {
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    var request =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    return drivetrain.applyRequest(
        () ->
            request
                .withVelocityX(-DriveInput.exponential(translationX.getAsDouble(), 3) * maxSpeed)
                .withVelocityY(-DriveInput.exponential(translationY.getAsDouble(), 3) * maxSpeed)
                .withRotationalRate(
                    -DriveInput.exponential(rotation.getAsDouble(), 2) * maxAngularRate));
  }
}
