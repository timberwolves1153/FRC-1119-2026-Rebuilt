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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOVortex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
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
  private final Indexer indexer;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOVortex());
        shooter = new Shooter(new ShooterIOTalonFX());

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
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
        indexer = new Indexer(new IndexerIOSim());
        shooter = new Shooter(new ShooterIOSim());
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
        indexer = new Indexer(new IndexerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    NamedCommands.registerCommand(
        "Start Intake", new InstantCommand(() -> intake.setIntakeVoltage(-12)));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.stopIntake()));
    NamedCommands.registerCommand(
        "Deploy Intake", new InstantCommand(() -> intake.setDeployVoltage(4)));
    NamedCommands.registerCommand(
        "Stop Deploy Intake", new InstantCommand(() -> intake.stopDeploy()));
    NamedCommands.registerCommand(
        "Bring in Intake", new InstantCommand(() -> intake.setDeployVoltage(-4)));
    NamedCommands.registerCommand(
        "Start Indexing", new InstantCommand(() -> indexer.setIndexVoltage(-6)));
    NamedCommands.registerCommand("Stop Indexing", new InstantCommand(() -> indexer.stopIndex()));
    NamedCommands.registerCommand(
        "Start Loading", new InstantCommand(() -> indexer.setLoadVoltage(-12)));
    NamedCommands.registerCommand("Stop Loading", new InstantCommand(() -> indexer.stopLoad()));
    NamedCommands.registerCommand(
        "Start Shooter", new InstantCommand(() -> shooter.setShooterVoltage(-12)));
    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> shooter.stopShooter()));

    // Configure the button bindings
    configureButtonBindings();
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
            () -> -driveController.getRightX(),
            () -> driveController.a().getAsBoolean()));

    // Lock to 0° when A button is held
    // driveController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driveController.getLeftY(),
    //             () -> -driveController.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // intake movement
    opController.povDown().onTrue(new InstantCommand(() -> intake.setDeployVoltage(1)));
    opController.povDown().onFalse(new InstantCommand(() -> intake.stopDeploy()));
    opController.povUp().onTrue(new InstantCommand(() -> intake.setDeployVoltage(-1)));
    opController.povUp().onFalse(new InstantCommand(() -> intake.stopDeploy()));

    // intake
    opController.leftTrigger().onTrue(new InstantCommand(() -> intake.setIntakeVoltage(-11)));
    opController.leftTrigger().onFalse(new InstantCommand(() -> intake.stopIntake()));

    opController.rightStick().onTrue(new InstantCommand(() -> intake.setIntakeVoltage(11)));
    opController.rightStick().onFalse(new InstantCommand(() -> intake.stopIntake()));

    // opController.leftTrigger().onTrue(new InstantCommand(() -> indexer.setIndexVoltage(4)));
    // opController.leftTrigger().onFalse(new InstantCommand(() -> indexer.stopIndex()));

    opController.rightStick().onTrue(new InstantCommand(() -> indexer.setIndexVoltage(6)));
    opController.rightStick().onFalse(new InstantCommand(() -> indexer.stopIndex()));

    // Shooting
    opController.rightTrigger().onTrue(new InstantCommand(() -> shooter.setShooterVoltage(-8)));
    opController.rightTrigger().onFalse(new InstantCommand(() -> shooter.stopShooter()));

    opController.rightTrigger().onTrue(new InstantCommand(() -> shooter.setShooterVoltage(-9)));
    opController.rightTrigger().onFalse(new InstantCommand(() -> shooter.stopShooter()));

    opController.rightTrigger().onTrue(new InstantCommand(() -> indexer.setIndexVoltage(-10)));
    opController.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopIndex()));

    opController.rightTrigger().onTrue(new InstantCommand(() -> indexer.setLoadVoltage(10)));
    opController.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopLoad()));

    driveController.start().onTrue(new InstantCommand(() -> drive.resetGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
