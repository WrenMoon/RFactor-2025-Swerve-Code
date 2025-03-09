package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
import frc.robot.Commands.*;
import frc.robot.Commands.Arm.*;
import frc.robot.Commands.CV.*;
import frc.robot.Commands.Transfer.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final armSubystem arm = new armSubystem();
  private final shooterSubsystem shooter = new shooterSubsystem();
  private final loaderSubsystem loader = new loaderSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final climberSubsystem climber = new climberSubsystem();
  final Joystick WakakeController = new Joystick(0);
  final CommandPS5Controller MarkRoberController = new CommandPS5Controller(1);
  final CommandPS5Controller driveController2 = new CommandPS5Controller(0);

  public RobotContainer() {

    // NamedCommands.registerCommand("intakeCmd", new intakeCmd(intake, loader, 0.8, 0.4));
    // NamedCommands.registerCommand("shootCmd", new shootCmd(shooter, loader, true, 1));
    // NamedCommands.registerCommand("Note3Ang", new armPID(arm, 0.057, false));
    // NamedCommands.registerCommand("resetArm", new armPID(arm, 0, false));
    // NamedCommands.registerCommand("iiylsd", new armPID(arm, 0.1, false));
    // NamedCommands.registerCommand("ArmIntake", new armPID(arm, Constants.Arm.MinPose, true));

    configureBindings();

    Command driveSwerve = swerve.driveCommand(
        () -> -MathUtil.applyDeadband(WakakeController.getRawAxis(1), Constants.ControllerDeadband),
        () -> -MathUtil.applyDeadband(WakakeController.getRawAxis(0), Constants.ControllerDeadband),
        () -> getAsInt(WakakeController.getRawButton(5)) - getAsInt(WakakeController.getRawButton(6)), false, true);

    // Command climb = new climberCmd(climber,
    //     () -> -MathUtil.applyDeadband(MarkRoberController.getLeftY(), Constants.ControllerDeadband),
    //     () -> -MathUtil.applyDeadband(MarkRoberController.getRightY(), Constants.ControllerDeadband));

    // climber.setDefaultCommand(climb);
    swerve.setDefaultCommand(driveSwerve);

  }

  private void configureBindings() {

    // new JoystickButton(WakakeController, 3).whileTrue(Commands.runOnce(swerve::zeroGyro));
    // new JoystickButton(WakakeController, 8).whileTrue(new intakeCmd(intake, loader, 1, 0.7));
    // new JoystickButton(WakakeController, 7).whileTrue(new intakeCmd(intake, loader, -1, 0));
    // new JoystickButton(WakakeController, 11).whileTrue(new NoteAlign(swerve));
    // new JoystickButton(WakakeController, 2).whileTrue(new SpeakerAlign(swerve));

    // MarkRoberController.R2().whileTrue(new armPID(arm, Constants.Arm.MaxPose, true));
    // MarkRoberController.L2().onTrue(new armPID(arm, Constants.Arm.MinPose, true));
    // MarkRoberController.cross().whileTrue(new shootCmd(shooter, loader, false, 1));
    // MarkRoberController.R1().whileTrue(new intakeCmd(intake, loader, 0, 0.8));
    // MarkRoberController.triangle().whileTrue(new shootCmd(shooter, loader, true, 1));

  }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("Three note");
  }

  public double getAsInt(boolean bollean) {
    if (bollean) {
      return 0.8;
    } else {
      return 0;
    }
  }

}
