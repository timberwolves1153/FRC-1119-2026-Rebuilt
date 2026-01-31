package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.lib.util.AxisButton;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driver = new CommandXboxController(0);//new Joystick(0);
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    //private final JoystickButton driveRightStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final Swerve s_Swerve = new Swerve();

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> true

    ));
    NamedCommands.registerCommand("Reset Gyro 0", new InstantCommand(() -> s_Swerve.zeroGyro(0), s_Swerve));
    NamedCommands.registerCommand("Reset Gyro 54", new InstantCommand(() -> s_Swerve.zeroGyro(54), s_Swerve));
    NamedCommands.registerCommand("Reset Gyro 60", new InstantCommand(() -> s_Swerve.zeroGyro(60), s_Swerve));
    NamedCommands.registerCommand("Reset Gyro 120", new InstantCommand(() -> s_Swerve.zeroGyro(120), s_Swerve));
    NamedCommands.registerCommand("Reset Gyro 126", new InstantCommand(() -> s_Swerve.zeroGyro(126), s_Swerve));
    NamedCommands.registerCommand("Reset Gyro 180", new InstantCommand(() -> s_Swerve.zeroGyro(180), s_Swerve));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    //driver.rightStick().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0)));
    driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0)));
  }

  public void resetEncoders() {
    s_Swerve.resetEncoders();
  }

  public void resetPosition() {
    s_Swerve.resetOdometry(Pose2d.kZero);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}