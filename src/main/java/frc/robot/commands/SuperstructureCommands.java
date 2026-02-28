package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public final class SuperstructureCommands {

  private final Drive drive;
  private final Intake intake;
  private final Floor floor;
  private final Feeder feeder;
  private final Launcher launcher;

  public SuperstructureCommands(
      Drive drive, Intake intake, Floor floor, Feeder feeder, Launcher launcher) {
    this.drive = drive;
    this.intake = intake;
    this.floor = floor;
    this.feeder = feeder;
    this.launcher = launcher;
  }

  public Command launchWhenReady() {
    PrepareLaunchCommand prepareCommand = new PrepareLaunchCommand(launcher, drive::getPose);
    return Commands.parallel(
        prepareCommand, Commands.waitUntil(() -> prepareCommand.isReadyToLaunch()).andThen(feed()));
  }

  public Command launchManually() {
    return launcher
        .dashboardSpinUpCommand()
        .andThen(feed())
        .handleInterrupt(() -> launcher.stopLauncher());
  }

  public Command feed() {
    return Commands.sequence(
        Commands.waitSeconds(0.25),
        Commands.parallel(
            feeder.feedCommand(),
            Commands.waitSeconds(0.125)
                .andThen(floor.floorCommand().alongWith(intake.agitateCommand()))));
  }

  public Command stop() {
    return Commands.parallel(
        intake.deployCommand(),
        new InstantCommand(() -> floor.stopFloor()),
        new InstantCommand(() -> feeder.stopFeed()),
        new InstantCommand(() -> launcher.stopLauncher()));
  }
}
