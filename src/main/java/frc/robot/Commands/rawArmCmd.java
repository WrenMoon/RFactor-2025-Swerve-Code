package frc.robot.Commands;

import frc.robot.Subsystems.armSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
    arm.setMotor(speed.getAsDouble());
    
    SmartDashboard.putBoolean("rawArmCmd", false);
    SmartDashboard.putNumber("Arm encoder", arm.getEncoder());
    SmartDashboard.putNumber("Arm speed", speed.getAsDouble());
    SmartDashboard.putNumber("Arm Degrees", arm.getDegrees());
  }

  @Override
  
  public void end(boolean interrupted) {
    arm.setMotor(0);
    SmartDashboard.putBoolean("rawArmCmd", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
