package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


import java.io.File;
import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final armSubsystem arm = new armSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  final Joystick WakakeController = new Joystick(0);
  // final Joystick AmaryanController = new Joystick(1);
  final CommandPS5Controller AmaryanController = new CommandPS5Controller(1);

  public RobotContainer() {

    // NamedCommands.registerCommand("intakeCmd", new intakeCmd(intake, loader, 0.8, 0.4));

    configureBindings();

    //Default Swerve Command to drive with 3 axis
    Command driveSwerve = swerve.driveCommand(
        () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(1) * ((WakakeController.getRawAxis(3)+ 1)/2) * Math.min((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband),
        () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(0) * ((WakakeController.getRawAxis(3)+ 1)/2) * Math.min((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband),
        () -> MathUtil.applyDeadband(-WakakeController.getRawAxis(4) * ((WakakeController.getRawAxis(3)+ 1)/2) * Math.min((1 - WakakeController.getRawAxis(2)), 0.2), Constants.ControllerDeadband), false, true); //Control heading with right joystick
        // () -> ((Controller1.getRawButton(5))? 1 : 0) - ((Controller1.getRawButton(6))? 1 : 0), false, true); //Control heading with bumpers

    //Default Elevator Command to move the elevator with one axis
    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(5), Constants.ControllerDeadband));

    //Default Arm Command to move the arm with one axis and gravity control
    Command moveArm = new rawArmCmd(arm,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(1), Constants.ControllerDeadband));

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
    SequentialCommandGroup L1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L1), new armPosCmd(arm, armPoses.L1, false)));
    SequentialCommandGroup L2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L2), new armPosCmd(arm, armPoses.L2, false)));
    SequentialCommandGroup L3 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L3), new armPosCmd(arm, armPoses.L2, false)));
    SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L4), new armPosCmd(arm, armPoses.L4, false)));
    SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, 0), new armPosCmd(arm, armPoses.zero, false));

    new JoystickButton(WakakeController, 4).onTrue(Commands.runOnce(swerve::zeroGyro));
    // new JoystickButton(AmaryanController, 1).whileTrue(new intakeCmd(intake, 0.2));
    // new JoystickButton(AmaryanController, 4).whileTrue(new intakeCmd(intake, -0.2));
    // new JoystickButton(AmaryanController, 2).whileTrue(new elevatorPosCmd(elevator, -100));
    // new JoystickButton(AmaryanController, 3).whileTrue(new armPosCmd(arm, 0, true));
    // new JoystickButton(AmaryanController, 5).onTrue(swerve.getAutonomousCommand("Test Auto"));
    AmaryanController.axisGreaterThan(3, Constants.ControllerDeadband).whileTrue(new intakeCmd(intake, AmaryanController.getR2Axis()));
    AmaryanController.axisGreaterThan(2, Constants.ControllerDeadband).whileTrue(new intakeCmd(intake, -AmaryanController.getL2Axis()));
    AmaryanController.povUp().onTrue(new armPosCmd(arm, armPoses.algae, false));
    AmaryanController.cross().onTrue(L2);
    AmaryanController.circle().onTrue(L3);
    AmaryanController.triangle().onTrue(L4);
    AmaryanController.square().onTrue(L1);
    AmaryanController.povDown().onTrue(L0);



  }

  /**
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("Test Auto");
  }
}

