// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFx;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorIO;
import frc.robot.subsystems.floor.FloorIOSim;
import frc.robot.subsystems.floor.FloorIOVortex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Position;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOTalonFX;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Floor floor;
  private final Feeder feeder;
  private final Launcher launcher;
  private final Limelight vision;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final SuperstructureCommands superstructureCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        GyroIOPigeon2 gyroIO = new GyroIOPigeon2();
        drive =
            new Drive(
                gyroIO,
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX());
        floor = new Floor(new FloorIOVortex());
        feeder = new Feeder(new FeederIOTalonFx());
        launcher = new Launcher(new LauncherIOTalonFX());
        vision = new Limelight("limelight");
        intake.setDeployMotorPosition(Position.HOMED.angle());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        floor = new Floor(new FloorIOSim());
        feeder = new Feeder(new FeederIOSim());
        launcher = new Launcher(new LauncherIO() {});
        vision = new Limelight("limelight");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        floor = new Floor(new FloorIO() {});
        feeder = new Feeder(new FeederIO() {});
        launcher = new Launcher(new LauncherIO() {});
        vision = new Limelight("limelight");
        break;
    }

    // Set up SysId routines

    // Configure Superstructure
    superstructureCommands = new SuperstructureCommands(drive, intake, floor, feeder, launcher);
    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("Start Collecting", intake.intakeCommand());
    NamedCommands.registerCommand("Stop Collecting", intake.stopIntakeCommand());
    NamedCommands.registerCommand("Start Firing", superstructureCommands.launchWhenReady());
    NamedCommands.registerCommand("Stop Firing", superstructureCommands.stop());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    vision.setDefaultCommand(updateVisionCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Lock to hub when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                drive::calculateAimingAngle));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Start button is pressed
    driveController.start().onTrue(new InstantCommand(() -> drive.resetGyro()));

    opController.leftTrigger().whileTrue(superstructureCommands.launchWhenReady());
    opController.leftTrigger().onFalse(superstructureCommands.stop());
    opController.rightTrigger().whileTrue(superstructureCommands.launchManually());
    opController.rightTrigger().onFalse(superstructureCommands.stop());
    opController.rightBumper().onTrue(intake.deployCommand());
    opController.leftBumper().onTrue(intake.retractCommand());
    opController.a().whileTrue(intake.intakeCommand());
    opController.a().onFalse(intake.stopIntakeCommand());
    opController.b().onTrue(intake.stopIntakeCommand());
    opController.x().onTrue(superstructureCommands.reverseCommand());
    opController.x().onFalse(superstructureCommands.stop());
    opController.rightStick().whileTrue(superstructureCommands.yeet());
    opController.rightStick().onFalse(superstructureCommands.stop());
    opController.povUp().whileTrue(intake.agitateCommand());
    opController.povDown().whileTrue(floor.floorCommand());
    opController.povRight().onTrue(intake.intakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void setGoalHub() {
    drive.setGoalHub(DriverStation.getAlliance());
  }

  private Command updateVisionCommand() {
    return vision
        .run(
            () -> {
              final Pose2d currentRobotPose = drive.getPose();
              final Optional<Limelight.Measurement> measurement =
                  vision.getMeasurement(currentRobotPose);
              measurement.ifPresent(
                  m -> {
                    drive.addVisionMeasurement(
                        m.poseEstimate.pose, m.poseEstimate.timestampSeconds, m.standardDeviations);
                  });
            })
        .ignoringDisable(true);
  }
}
