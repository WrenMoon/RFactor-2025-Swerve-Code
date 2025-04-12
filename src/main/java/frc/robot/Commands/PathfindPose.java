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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.*;


public class pathfindPose extends Command {
    private final SwerveSubsystem swerve;
    private final boolean blueAlliance;
    private final boolean leftAlign;
    private DoubleSupplier HeadingX;
    private DoubleSupplier HeadingY;
    private Pose2d targetPose;
    private boolean poseUpdated;

    public pathfindPose(SwerveSubsystem swerve, boolean blueAlliance, boolean leftAlign, DoubleSupplier HeadingX,
            DoubleSupplier HeadingY) {
        this.swerve = swerve;
        this.blueAlliance = blueAlliance;
        this.leftAlign = leftAlign;
        this.HeadingX = HeadingX;
        this.HeadingY = HeadingY;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        poseUpdated = false;
    }

    @Override
    public void execute() {

        if (blueAlliance) {

            if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == -1) {
                targetPose = Constants.PosesBlue.stationLeft;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == 1) {
                targetPose = Constants.PosesBlue.stationRight;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 1 && HeadingX.getAsDouble() == 0) {
                targetPose = leftAlign ? Constants.PosesBlue.reef1l : Constants.PosesBlue.reef1r;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 0.5773502691896258 && HeadingX.getAsDouble() == -1) {
                targetPose = leftAlign ? Constants.PosesBlue.reef2l : Constants.PosesBlue.reef2r;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == -0.5773502691896258 && HeadingX.getAsDouble() == -1) {
                targetPose = leftAlign ? Constants.PosesBlue.reef3l : Constants.PosesBlue.reef3r;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == -1 && HeadingX.getAsDouble() == 0) {
                targetPose = leftAlign ? Constants.PosesBlue.reef4l : Constants.PosesBlue.reef4r;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == -0.5773502691896258 && HeadingX.getAsDouble() == 1) {
                targetPose = leftAlign ? Constants.PosesBlue.reef5l : Constants.PosesBlue.reef5r;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 0.5773502691896258 && HeadingX.getAsDouble() == 1) {
                targetPose = leftAlign ? Constants.PosesBlue.reef6l : Constants.PosesBlue.reef6r;
                poseUpdated = true;
            }

        } else {

            if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == -1) {
                targetPose = Constants.PosesRed.stationLeft;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 0.7 && HeadingX.getAsDouble() == 1) {
                targetPose = Constants.PosesRed.stationRight;
                poseUpdated = true;
            } else if (HeadingY.getAsDouble() == 1 && HeadingX.getAsDouble() == 0) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef1l : Constants.PosesRed.reef1r;
            } else if (HeadingY.getAsDouble() == 0.5773502691896258 && HeadingX.getAsDouble() == -1) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef2l : Constants.PosesRed.reef2r;
            } else if (HeadingY.getAsDouble() == -0.5773502691896258 && HeadingX.getAsDouble() == -1) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef3l : Constants.PosesRed.reef3r;
            } else if (HeadingY.getAsDouble() == -1 && HeadingX.getAsDouble() == 0) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef4l : Constants.PosesRed.reef4r;
            } else if (HeadingY.getAsDouble() == -0.5773502691896258 && HeadingX.getAsDouble() == 1) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef5l : Constants.PosesRed.reef5r;
            } else if (HeadingY.getAsDouble() == 0.5773502691896258 && HeadingX.getAsDouble() == 1) {
                poseUpdated = true;
                targetPose = leftAlign ? Constants.PosesRed.reef3l : Constants.PosesRed.reef3r;
            }

        }
        if (poseUpdated) {
            // CommandScheduler.getInstance().schedule(swerve.driveToPose(() -> targetPose, 0));
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(swerve.driveToPose(() -> targetPose, 0), new alignPose(swerve, targetPose, HeadingX.getAsDouble(), HeadingY.getAsDouble())));
            SmartDashboard.putBoolean("Pose Align/Aligned", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
