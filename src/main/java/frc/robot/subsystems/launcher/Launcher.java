package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  public LauncherIO launcherIO;
  public LauncherInputsAutoLogged launcherInputs;

  private double YEET_RPM = -6000;

  public Launcher(LauncherIO launcherIO) {
    this.launcherIO = launcherIO;
    this.launcherInputs = new LauncherInputsAutoLogged();

    SmartDashboard.putNumber("LauncherSpeedRPM", 3750);
  }

  @Override
  public void periodic() {
    launcherIO.updateInputs(launcherInputs);
    Logger.processInputs("launcher", launcherInputs);
  }

  public void setLauncherVoltage(double volts) {
    launcherIO.setLauncherVoltage(volts);
  }

  public void setLauncherRPM(double rpm) {
    launcherIO.setLauncherRPM(rpm);
  }

  public Command reverseMotor() {
    return runOnce(() -> setLauncherRPM(2000));
  }

  public void stopLauncher() {
    launcherIO.stopLauncher();
  }

  public boolean isVelocityInTolerance() {
    return launcherIO.isVelocityInTolerance();
  }

  public Command spinUpCommand(double rpm) {
    return runOnce(() -> setLauncherRPM(rpm))
        .andThen(Commands.waitUntil(launcherIO::isVelocityInTolerance));
  }

  public Command dashboardSpinUpCommand() {
    return defer(() -> spinUpCommand(-SmartDashboard.getNumber("LauncherSpeedRPM", 3750)));
  }

  public Command yeetCommand() {
    return runOnce(() -> setLauncherRPM(YEET_RPM))
        .andThen(Commands.waitUntil(launcherIO::isVelocityInTolerance));
  }

  public Command stopLaunchCommand() {
    return runOnce(() -> setLauncherRPM(0));
  }
}
