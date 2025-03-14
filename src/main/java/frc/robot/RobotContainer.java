package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import static edu.wpi.first.units.Units.Meter;


import java.io.File;
import frc.robot.Commands.*;
import frc.robot.Commands.pathfindAlign.leftAlign;
import frc.robot.Commands.pathfindAlign.rightAlign;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final armSubsystem arm = new armSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  // final CommandXboxController WakakeController = new CommandXboxController(0);
  final CommandPS5Controller WakakeController = new CommandPS5Controller(0);
  final CommandPS5Controller AmaryanController = new CommandPS5Controller(1);

  public RobotContainer() {

    configureBindings();

    //Default Swerve Command to drive with 3 axis on Joystick
    // Command driveSwerve = swerve.driveCommand(
    //     () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(1) * ((WakakeController.getRawAxis(3)+ 2)/3) * Math.max((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband),
    //     () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(0) * ((WakakeController.getRawAxis(3)+ 2)/3) * Math.max((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband),
    //     () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(4) * ((WakakeController.getRawAxis(3)+ 2)/3) * Math.max((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband), false, true); //Control heading with right joystick
        // () -> ((Controller1.getRawButton(5))? 1 : 0) - ((Controller1.getRawButton(6))? 1 : 0), false, true); //Control heading with bumpers

    //Default Swerve Command to drive with 3 axis on XboxController
    // Command driveSwerve = swerve.driveCommand(
    //     () -> MathUtil.applyDeadband(-WakakeController.getLeftX() * ((WakakeController.getRightTriggerAxis() + 2)/3) * Math.max((1 - WakakeController.getLeftTriggerAxis()), 0.2), Constants.ControllerDeadband),
    //     () -> MathUtil.applyDeadband(-WakakeController.getLeftY() * ((WakakeController.getRightTriggerAxis() + 2)/3) * Math.max((1 - WakakeController.getLeftTriggerAxis()), 0.2), Constants.ControllerDeadband),
    //     () -> MathUtil.applyDeadband(-WakakeController.getRightX() * ((WakakeController.getRightTriggerAxis() + 2)/3) * Math.max((1 - WakakeController.getLeftTriggerAxis()), 0.2), Constants.ControllerDeadband), false, true); //Control heading with right joystick

     //Default Swerve Command to drive with 3 axis on PS5 Controller
     Command driveSwerve = swerve.driveCommand(
      () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getRightX() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband), false, true); //Control heading with right joystick

    //Default Elevator Command to move the elevator with one axis
    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(5), Constants.ControllerDeadband));

    //Default Arm Command to move the arm with one axis and gravity control
    Command moveArm = new rawArmCmd(arm,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(1) * 0.35, Constants.ControllerDeadband));

    //Default Intake Command ot move the intake with gravity correction as per degree angle of the arm
    Command holdCoral = intake.setMotorSupplier(
        () -> Math.cos(Math.toRadians(arm.getDegrees() - Constants.Intake.angleOffset)) * -Constants.Intake.Kg);

    //Applying all the default commands
    elevator.setDefaultCommand(elevate);
    swerve.setDefaultCommand(driveSwerve);
    arm.setDefaultCommand(moveArm);
    intake.setDefaultCommand(holdCoral);
  }

  /**
   * Configuring all the button bindings for all the joysticks
   */
  private void configureBindings() {

    Constants.Arm.poses armPoses = new Constants.Arm.poses();
    Constants.Elevator.poses elevatorPoses = new Constants.Elevator.poses();

    //creating command groups for depositing corals
    SequentialCommandGroup L1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L1), new armPosCmd(arm, armPoses.L1, false));
    SequentialCommandGroup L2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L2));
    SequentialCommandGroup L3 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L3));
    SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L4));
    // SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, 0), new armPosCmd(arm, armPoses.zero, false));
    SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, 0), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.zero, false));
    

    NamedCommands.registerCommand("L4", L4);

    // WakakeController.y().onTrue(Commands.runOnce(swerve::zeroGyro));
    // WakakeController.rightBumper().whileTrue(new rightAlign(swerve));
    // WakakeController.leftBumper().whileTrue(new leftAlign(swerve));
    WakakeController.triangle().onTrue(Commands.runOnce(swerve::zeroGyro));
    // WakakeController.R1().whileTrue(new rightAlign(swerve));
    // WakakeController.R1().onTrue(swerve.driveToPose(new Pose2d(new Translation2d(Meter.of(5), Meter.of(5)), Rotation2d.fromDegrees(0)),0));
    // WakakeController.L1().whileTrue(new leftAlign(swerve));

    AmaryanController.R2().whileTrue(new intakeCmd(intake, 0.3));
    AmaryanController.L2().whileTrue(new intakeCmd(intake, 0.6));
    AmaryanController.R1().whileTrue(new intakeCmd(intake, -0.3));
    AmaryanController.L1().whileTrue(new intakeCmd(intake, -0.1));
    AmaryanController.povUp().onTrue(new armPosCmd(arm, armPoses.algae, false));
    AmaryanController.cross().onTrue(L2);
    AmaryanController.circle().onTrue(L3);
    AmaryanController.triangle().onTrue(L4);
    AmaryanController.square().onTrue(L1);
    AmaryanController.povDown().onTrue(L0);

    AmaryanController.povLeft().whileTrue(new armPosCmd(arm, 0, true));
  }

  /**
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("Test Auto");
  }
}

