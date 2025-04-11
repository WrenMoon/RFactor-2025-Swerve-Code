package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.armSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class armPosCmd extends Command {
    private final armSubsystem arm;
    private final double targetPose;
    private final boolean holdPID;
    private PIDController PIDarm;
    private boolean endLoop = false;

    /**
     * A command to move the arm to an encoder setpoint using PID Feedback and
     * Gravity compensation feedforward.
     * 
     * @param arm        the arm subsystem to move
     * @param targetPose the target pose in dergees to move to
     * @param holdPID    whether or not to hold the PID loop after acceptable error
     *                   is achieved
     */
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
        PIDarm.setSetpoint(targetPose); // PID setpoint
    }

    @Override
    public void execute() {

        double speed = PIDarm.calculate(arm.getDegrees()); // PID Correction value
        speed = speed + Constants.Arm.Kg * Math.cos(Math.toRadians(arm.getDegrees())); // Feedforward Gravity compensation

        speed = speed + ((arm.getDegrees() > 80) ? -0.014 : 0); // play correction
        speed = speed + ((arm.getDegrees() > 93) ? 0.007 : 0); // play correction

        speed = Math.min(Math.max(speed, -Constants.Arm.MaxSpeed), Constants.Arm.MaxSpeed); // Applying Speed Limits

        // Smartdashboard for debugging
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("Arm/armPosCmd", true);
            SmartDashboard.putNumber("Arm/Arm encoder", arm.getEncoder());
            SmartDashboard.putNumber("Arm/Arm Target Pose", targetPose);
            SmartDashboard.putNumber("Arm/Arm PID speed", speed);
            SmartDashboard.putNumber("Arm/Arm Degrees", arm.getDegrees());
            SmartDashboard.putBoolean("Arm/Arm hold", holdPID);
        }

        if (Math.abs(targetPose - arm.getDegrees()) < 3.5 && !holdPID) { // endcase when setpoint achieved. Only if holdPID is false
            endLoop = true;
        }

        arm.setMotor(speed); // applies the speed to the motor
    }

    @Override
    public void end(boolean interrupted) {
        arm.setMotor(0); // stop the motor when the command is stopped

        // Smartdashboard for debugging
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("Arm/armPosCmd", false);
        }
    }

    @Override
    public boolean isFinished() {
        return endLoop; // ends the command when setpoint is reached and holdPID is false
    }
}
