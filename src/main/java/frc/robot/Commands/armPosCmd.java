package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.armSubsystem;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class armPosCmd extends Command {
    private final armSubsystem arm;
    private final double targetPose;
    private final boolean holdPID;
    private PIDController PIDarm;
    private boolean endLoop = false;

    public armPosCmd(armSubsystem arm, double targetPose, boolean holdPID) {
        this.arm = arm;
        this.targetPose = targetPose;
        this.holdPID = holdPID;
        PIDarm = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        endLoop = false;
        PIDarm.setSetpoint(targetPose);
    }

    @Override
    public void execute() {

        double speed = PIDarm.calculate(arm.getEncoder());
        speed = Math.min(Math.max(speed, -1), 1);
        arm.setMotor(speed);

        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("armPosCmd", true);
            SmartDashboard.putNumber("Arm encoder", arm.getEncoder());
            SmartDashboard.putNumber("Arm Target Pose", targetPose);
            SmartDashboard.putNumber("Arm PID speed", speed);
            SmartDashboard.putNumber("Arm Degrees", arm.getDegrees());
            SmartDashboard.putBoolean("Arm hold", holdPID);
        }

        if (speed < 0.007) {
            if (holdPID) {
                PIDarm = new PIDController(Constants.Arm.holdKp, 0, Constants.Arm.holdKd);
            } else {
                endLoop = true;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        arm.setMotor(0);
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("armPosCmd", false);
        }
    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }
}
