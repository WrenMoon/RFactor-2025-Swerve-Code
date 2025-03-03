package frc.robot.Commands.Arm;

import frc.robot.Subsystems.armSubystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class armCmd extends Command {
  private final armSubystem Arm;
  private double targetPose;
  private PIDController PIDarm;

  public armCmd(armSubystem subsystem, double targetPose) {
    this.Arm = subsystem;
    this.targetPose = targetPose;
    this.PIDarm = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

    PIDarm.setSetpoint(targetPose);
  }

  @Override
  public void execute() {

    
    Arm.setRawMotors(targetPose, targetPose);

    if (Constants.smartEnable) {
      SmartDashboard.putNumber("Arm Position", Arm.getAbsoluteEncoder());
      SmartDashboard.putNumber("Raw Arm Position", Arm.getRawEncoder());
      SmartDashboard.putBoolean("armCmd", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Arm.setMotors(0);
    SmartDashboard.putBoolean("armCmd", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
