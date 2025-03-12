package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final armSubsystem arm = new armSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  final Joystick Controller1 = new Joystick(0);
  final Joystick Controller2 = new Joystick(1);
  // final CommandPS5Controller MarkRoberController = new CommandPS5Controller(1);

  public RobotContainer() {

    // NamedCommands.registerCommand("intakeCmd", new intakeCmd(intake, loader, 0.8, 0.4));

    configureBindings();

    Command driveSwerve = swerve.driveCommand(
        () -> -MathUtil.applyDeadband(Controller1.getRawAxis(1), Constants.ControllerDeadband),
        () -> -MathUtil.applyDeadband(Controller1.getRawAxis(0), Constants.ControllerDeadband),
        () -> -MathUtil.applyDeadband(Controller1.getRawAxis(4), Constants.ControllerDeadband), false, true);

    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(Controller2.getRawAxis(5), Constants.ControllerDeadband));

    Command moveArm = new rawArmCmd(arm,
        () -> -MathUtil.applyDeadband(Controller2.getRawAxis(1), Constants.ControllerDeadband));

    Command holdCoral = intake.setMotorSupplier(
        () -> Math.cos(Math.toRadians(arm.getDegrees() - Constants.Intake.angleOffset)) * -Constants.Intake.Kg);

    elevator.setDefaultCommand(elevate);
    swerve.setDefaultCommand(driveSwerve);
    arm.setDefaultCommand(moveArm);
    intake.setDefaultCommand(holdCoral);
  }

  private void configureBindings() {
    new JoystickButton(Controller2, 1).whileTrue(new intakeCmd(intake, 0.2));
    new JoystickButton(Controller2, 4).whileTrue(new intakeCmd(intake, -0.2));
    new JoystickButton(Controller2, 2).whileTrue(new elevatorPosCmd(elevator, -100));
    new JoystickButton(Controller2, 3).whileTrue(new armPosCmd(arm, 0, true));
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
