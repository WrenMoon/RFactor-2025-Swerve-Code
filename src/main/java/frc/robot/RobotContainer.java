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
  // private final shooterSubsystem shooter = new shooterSubsystem();
  // private final loaderSubsystem loader = new loaderSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  final Joystick Controller1 = new Joystick(0);
  final Joystick Controller2 = new Joystick(1);
  // final CommandPS5Controller MarkRoberController = new CommandPS5Controller(1);
  // final CommandPS5Controller driveController2 = new CommandPS5Controller(0);

  public RobotContainer() {

    // NamedCommands.registerCommand("intakeCmd", new intakeCmd(intake, loader, 0.8, 0.4));
    // NamedCommands.registerCommand("shootCmd", new shootCmd(shooter, loader, true, 1));
    // NamedCommands.registerCommand("Note3Ang", new armPID(arm, 0.057, false));
    // NamedCommands.registerCommand("resetArm", new armPID(arm, 0, false));
    // NamedCommands.registerCommand("iiylsd", new armPID(arm, 0.1, false));
    // NamedCommands.registerCommand("ArmIntake", new armPID(arm, Constants.Arm.MinPose, true));

    configureBindings();

    Command driveSwerve = swerve.driveCommand(
        () -> -MathUtil.applyDeadband(Controller1.getRawAxis(1), Constants.ControllerDeadband),
        () -> -MathUtil.applyDeadband(Controller1.getRawAxis(0), Constants.ControllerDeadband),
        () -> getAsInt(Controller1.getRawButton(5)) - getAsInt(Controller1.getRawButton(6)), false, true);

    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(Controller2.getRawAxis(1), Constants.ControllerDeadband),
        () -> -MathUtil.applyDeadband(Controller2.getRawAxis(5), Constants.ControllerDeadband));

    Command moveArm = new rawArmCmd(arm, 
        () -> -MathUtil.applyDeadband(Controller2.getRawAxis(2) - Controller2.getRawAxis(3), Constants.ControllerDeadband));

    elevator.setDefaultCommand(elevate);
    swerve.setDefaultCommand(driveSwerve);
    arm.setDefaultCommand(moveArm);
  }

  private void configureBindings() {

    // new JoystickButton(Controller1, 3).whileTrue(Commands.runOnce(swerve::zeroGyro));
    new JoystickButton(Controller2, 1).whileTrue(new intakeCmd(intake, 0.5));
    new JoystickButton(Controller2, 2).whileTrue(new intakeCmd(intake, -0.5));
    // new JoystickButton(Controller1, 11).whileTrue(new NoteAlign(swerve));
    // new JoystickButton(Controller1, 2).whileTrue(new SpeakerAlign(swerve));

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
