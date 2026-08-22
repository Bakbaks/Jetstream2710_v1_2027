package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommands;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotConstants;
import frc.robot.shooting.ShotTable;
import frc.robot.shooting.ShotVerifier;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {
  private static final int DRIVER_PORT = 0;
  private static final int AUX_PORT = 1;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final RobotState robotState =
      new RobotState(drivetrain::getPose, drivetrain::getFieldRelativeSpeeds);
  private final Shooter shooter =
      new Shooter(RobotBase.isReal() ? new ShooterIOReal() : new ShooterIOSim());
  private final Hopper hopper =
      new Hopper(RobotBase.isReal() ? new HopperIOReal() : new HopperIOSim());
  private final Intake intake =
      new Intake(RobotBase.isReal() ? new IntakeIOReal() : new IntakeIOSim());
  private final Vision vision =
      new Vision(
          RobotBase.isReal() ? new VisionIOReal() : new VisionIOSim(),
          drivetrain::addVisionMeasurement);
  private final ShotCalculator shotCalculator = new ShotCalculator(robotState, new ShotTable());
  private final ShotVerifier shotVerifier = new ShotVerifier(robotState, shooter);
  private final CommandGamepad driverController = new CommandGamepad(DRIVER_PORT);
  private final CommandGamepad auxController = new CommandGamepad(AUX_PORT);
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        DriveCommands.joystickDrive(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX));
    driverController.dpadDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
            Commands.startEnd(
                () -> {
                  shooter.setShotRPM(ShooterConstants.DEFAULT_SHOT_RPM);
                  shooter.setGoal(Shooter.Goal.SHOOT);
                },
                () -> shooter.setGoal(Shooter.Goal.STOP),
                shooter)));
    driverController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                hopper.goalCommand(Hopper.Goal.HOLD, Hopper.Goal.HOLD),
                intake.goalCommand(Intake.Goal.INTAKE, Intake.Goal.DEPLOY)));
    
    auxController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                intake.goalCommand(Intake.Goal.COMPRESSION, Intake.Goal.STOW)
            )
        );
    
    
  }



  //Autonomous Settings

  private void configureAutos() {
    NamedCommands.registerCommand(
        "Test",
        Commands.run(
                () -> {
                  var shotSolution = shotCalculator.calculate(getAllianceTarget());
                  shooter.setShotRPM(shotSolution.shooterRPM());
                  shooter.setGoal(Shooter.Goal.SHOOT);
                  hopper.setGoal(
                      shotVerifier.canFire(shotSolution) ? Hopper.Goal.FEED : Hopper.Goal.HOLD);
                },
                shooter,
                hopper)
            .withTimeout(2.0));

    autoChooser = AutoBuilder.buildAutoChooser("Taxi");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Optional<Translation2d> getAllianceTarget() {
    int targetTagId =
        MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED
            ? ShotConstants.RED_TARGET_TAG_ID
            : ShotConstants.BLUE_TARGET_TAG_ID;
    return ShotConstants.FIELD_LAYOUT
        .getTagPose(targetTagId)
        .map(
            tagPose ->
                tagPose
                    .toPose2d()
                    .plus(ShotConstants.TAG_TO_TARGET)
                    .getTranslation());
  }

  

  public void periodic() {
    robotState.periodic();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
