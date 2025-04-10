package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PathfindPose extends Command {
    private final SwerveSubsystem swerve;
    private final boolean blueAlliance;
    private final boolean leftAlign;
    private DoubleSupplier HeadingX;
    private DoubleSupplier HeadingY;
    private Pose2d targetPose;

    /**
     * A command to move the swerve horizontally to align it to an april tag
     * 
     * @param swerve the swerve subsystem to move
     */
    public PathfindPose(SwerveSubsystem swerve, boolean blueAlliance, boolean leftAlign, DoubleSupplier HeadingX, DoubleSupplier HeadingY) {
        this.swerve = swerve;
        this.blueAlliance = blueAlliance;
        this.leftAlign = leftAlign;
        this.HeadingX = HeadingX;
        this.HeadingY = HeadingY;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {

        targetPose = Constants.PosesBlue.stationLeft;
    
        if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == -1) {
            targetPose = Constants.PosesBlue.stationLeft;
        } else if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == 1) {
            targetPose = Constants.PosesBlue.stationRight;
        } else if (HeadingY.getAsDouble() == 1 && HeadingX.getAsDouble() == 0) {
            targetPose = leftAlign ? Constants.PosesBlue.reef1l : Constants.PosesBlue.reef1r;
        } else if (HeadingY.getAsDouble() == 0.5773502691896258 && HeadingX.getAsDouble() == -1) {
            targetPose = leftAlign ? Constants.PosesBlue.reef2l : Constants.PosesBlue.reef2r;
        } else if (HeadingY.getAsDouble() == -0.5773502691896258 && HeadingX.getAsDouble() == -1) {
            targetPose = leftAlign ? Constants.PosesBlue.reef3l : Constants.PosesBlue.reef3r;
    
        }
        SmartDashboard.putNumber("Test X", HeadingX.getAsDouble());
        SmartDashboard.putNumber("Test Y", HeadingY.getAsDouble());


        CommandScheduler.getInstance().schedule(swerve.driveToPose(() -> targetPose, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false); // Stop the swerve when the command is stopped
    }

    @Override
    public boolean isFinished() {
        return false; // End the command when the setpoint is achieved or if the limelight isnt seeing
                      // anything
    }
}
