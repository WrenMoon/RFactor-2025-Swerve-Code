package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class reefAlign extends Command {
    private final SwerveSubsystem swerve;
    private final boolean holdPID;
    private final double targetAngle;
    private PIDController PIDcv;
    private boolean endLoop;

    /**
     * A command to move the swerve horizontally to align it to an april tag
     * 
     * @param swerve      the swerve subsystem to move
     * @param holdPID     whether to hold the PID after reaching the setpoint
     * @param targetAngle the target TX angle from the april tag
     */
    public reefAlign(SwerveSubsystem swerve, boolean holdPID, double targetAngle) {
        this.swerve = swerve;
        this.holdPID = holdPID;
        this.targetAngle = targetAngle;
        PIDcv = new PIDController(Constants.CV.kp, 0, Constants.CV.kd);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endLoop = false;
        PIDcv.setSetpoint(targetAngle); // PID setpoint
    }

    @Override
    public void execute() {

        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");

        if (llresults != null && LimelightHelpers.getTX("limelight") != 0) { // Check if the limelight is returning any values

            double speed = PIDcv.calculate(LimelightHelpers.getTX("limelight")); // Calculate the PID correction
            speed = Math.min(Math.max(speed, -Constants.CV.MaxSpeed), Constants.CV.MaxSpeed); // Applying Speed Limits

            if ((Math.abs(targetAngle - LimelightHelpers.getTX("limelight")) < 0.5) && !holdPID) { // endcase when setpoint achieved. Only if holdPID is false
                endLoop = true;
            }

            swerve.drive(new Translation2d(0.07, speed), 0, false); // Drive the swerve to align it

            // Smartdashboard for debuggign
            if (Constants.smartEnable) {
                SmartDashboard.putNumber("Reef Tag TX", LimelightHelpers.getTX("limelight"));
                SmartDashboard.putNumber("Reef Align Correction", speed);
                SmartDashboard.putBoolean("ReefAlign", true);
            }
        } else {
            endLoop = true; // Stop the command if the limelight isnt seeing anything
            swerve.drive(new Translation2d(0, 0), 0, false); // Stop the swerve when the command is stopped
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false); // Stop the swerve when the command is stopped
        if (Constants.smartEnable){
            SmartDashboard.putBoolean("ReefAlign", false);
        }
    }

    @Override
    public boolean isFinished() {
        return endLoop; // End the command when the setpoint is achieved or if the limelight isnt seeing anything
    }
}
