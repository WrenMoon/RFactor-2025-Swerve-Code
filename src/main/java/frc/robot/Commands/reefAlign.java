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
     * A command to move the swerve to an encoder setpoint using PID Feedback and
     * Gravity compensation feedforward.
     * 
     * @param swerve     the swerve subsystem to move
     * @param targetPose the target pose in dergees to move to
     * @param holdPID    whether or not to hold teh PID loop after acceptable error
     *                   is achieved
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

        if (llresults != null) {

            double speed = PIDcv.calculate(LimelightHelpers.getTX("limelight"));
            speed = Math.min(Math.max(speed, -Constants.CV.MaxSpeed), Constants.CV.MaxSpeed); //Applying Speed Limits
            

            if ((Math.abs(targetAngle - LimelightHelpers.getTX("limelight")) < 0.5) && !holdPID) {
                endLoop = true;
            }

            // swerve.drive(new Translation2d(0, speed), 0, false);

            if (Constants.smartEnable) {
                SmartDashboard.putNumber("Reef Tag TX", LimelightHelpers.getTX("limelight"));

                SmartDashboard.putNumber("Reef Align Correction", speed);
                SmartDashboard.putBoolean("ReefAlign", true);
            }
        } else {
            endLoop = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }
}
