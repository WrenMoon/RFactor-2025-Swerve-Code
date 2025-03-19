package frc.robot.Commands;

import frc.robot.Subsystems.armSubsystem;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class rawArmCmd extends Command {
  private final armSubsystem arm;
  private DoubleSupplier speed;
  private double finalSpeed = 0;

  /**
   * A command to move the arm with at a certain power along with gravity
   * compensation feedforawrd.
   * 
   * @param subsystem the arm subsystem to move
   * @param speed     the speed to move the arm at
   */
  public rawArmCmd(armSubsystem subsystem, DoubleSupplier speed) {
    this.arm = subsystem;
    this.speed = speed;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    finalSpeed = 0;
  }

  @Override
  public void execute() {

    if ((speed.getAsDouble() > 0 && speed.getAsDouble() < Constants.Arm.poses.maxPose)
        || (speed.getAsDouble() < 0 && speed.getAsDouble() > Constants.Arm.poses.minPose)) {
      finalSpeed =(speed.getAsDouble() + Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees()))); // Apply the motor speed correction
    } else{
      finalSpeed =(Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees())));
    }

    finalSpeed = finalSpeed + ((arm.getDegrees() > 80) ? -0.014 : 0); // play correction
    finalSpeed = finalSpeed + ((arm.getDegrees() > 93) ? 0.007 : 0); // play correction



    arm.setMotor(finalSpeed);

    // Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawArmCmd", false);
      SmartDashboard.putNumber("Arm encoder", arm.getEncoder());
      SmartDashboard.putNumber("Arm speed", speed.getAsDouble());
      SmartDashboard.putNumber("Arm Degrees", arm.getDegrees());
      SmartDashboard.putNumber("Arm Correction", Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees())));
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.setMotor(0); // Stop the motor when the command is stopped

    // Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawArmCmd", false);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interupted by the command scheduler
  }
}
