package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import org.wpilib.framework.RobotBase;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;//verify is it CommandGamepad or CommandJoystick or CommandNiDsXboxController
import frc.robot.commands.DriveCommands;
import frc.robot.power.PowerMonitor;
import frc.robot.shooting.ShotCalculator;
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
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.Goal;

public class RobotContainer {
  private static final int DRIVER_PORT = 0;

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
  private final Superstructure superstructure =
      new Superstructure(shooter, hopper, intake, shotCalculator, shotVerifier);
  private final PowerMonitor powerMonitor = new PowerMonitor();

  private final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(DRIVER_PORT);
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
    driverController.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverController.rightTrigger().whileTrue(superstructure.goalCommand(Goal.SHOOT));
    driverController.leftTrigger().whileTrue(superstructure.goalCommand(Goal.INTAKE));
    driverController.leftBumper().whileTrue(superstructure.goalCommand(Goal.EJECT));
    driverController.a().whileTrue(superstructure.goalCommand(Goal.PASS));
    driverController.y().whileTrue(superstructure.goalCommand(Goal.PREP_SHOT));
    driverController.b().onTrue(superstructure.setGoalCommand(Goal.IDLE));
  }

  private void configureAutos() {
    NamedCommands.registerCommand("PreloadVolley", timedGoal(Goal.SHOOT, 2.0));
    NamedCommands.registerCommand("ExtraVolley", timedGoal(Goal.SHOOT, 5.0));
    NamedCommands.registerCommand("BackMoveVolley", timedGoal(Goal.SHOOT, 4.2));
    NamedCommands.registerCommand("FinalMoveVolley", timedGoal(Goal.SHOOT, 8.0));
    NamedCommands.registerCommand("QuickVolley", timedGoal(Goal.SHOOT, 3.0));
    NamedCommands.registerCommand("SpinIntake", timedGoal(Goal.INTAKE, 8.0));
    NamedCommands.registerCommand("UnJamKickBar", timedGoal(Goal.EJECT, 0.5));
    NamedCommands.registerCommand(
        "ExtendIntake", Commands.runOnce(() -> superstructure.setGoal(Goal.INTAKE)));
    NamedCommands.registerCommand(
        "RetractIntake", Commands.runOnce(() -> superstructure.setGoal(Goal.IDLE)));

    autoChooser = AutoBuilder.buildAutoChooser("Taxi");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Command timedGoal(Goal goal, double seconds) {
    return Commands.run(() -> superstructure.setGoal(goal))
        .finallyDo(() -> superstructure.setGoal(Goal.IDLE))
        .withTimeout(seconds);
  }

  public void periodic() {
    robotState.periodic();
    powerMonitor.periodic();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
