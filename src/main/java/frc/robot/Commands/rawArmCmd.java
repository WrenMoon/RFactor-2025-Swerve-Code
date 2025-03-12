package frc.robot.Commands;

import frc.robot.Subsystems.armSubsystem;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//A command to move the arm with at a certain power along with gravity compensation feedforawrd.

public class rawArmCmd extends Command {
  private final armSubsystem arm;
  private DoubleSupplier speed;

  public rawArmCmd(armSubsystem subsystem, DoubleSupplier speed) {
    this.arm = subsystem;
    this.speed = speed;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    arm.setMotor(speed.getAsDouble() + Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees()))); //Apply the motor speed with the gravity correction

    //Smartdashboard for debugging
    if (Constants.smartEnable){
      SmartDashboard.putBoolean("rawArmCmd", false);
      SmartDashboard.putNumber("Arm encoder", arm.getEncoder());
      SmartDashboard.putNumber("Arm speed", speed.getAsDouble());
      SmartDashboard.putNumber("Arm Degrees", arm.getDegrees());
      SmartDashboard.putNumber("Arm Correction", Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees())));
    }
  }

  @Override

  public void end(boolean interrupted) {
    arm.setMotor(0);

    //Smartdashboard for debugging
    if (Constants.smartEnable){
      SmartDashboard.putBoolean("rawArmCmd", false);
    }
  }

  @Override
  public boolean isFinished() {
    return false; //runs until interupted by the command scheduler
  }
}
