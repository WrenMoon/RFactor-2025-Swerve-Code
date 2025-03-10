package frc.robot.Commands.Arm;

import frc.robot.Subsystems.armSubystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class rawArmCmd extends Command {
  private final armSubystem Arm;
  private DoubleSupplier speed;

  public rawArmCmd(armSubystem subsystem, DoubleSupplier speed) {
    this.Arm = subsystem;
    this.speed = speed;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    Arm.setRawMotors(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Arm.setRawMotors(0);
    SmartDashboard.putBoolean("armCmd", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
