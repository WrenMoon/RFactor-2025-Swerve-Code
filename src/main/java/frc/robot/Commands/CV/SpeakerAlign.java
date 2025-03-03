package frc.robot.Commands.CV;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.SwerveSubsystem;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class SpeakerAlign extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController PIDang;
    private final PIDController PIDmove;
    public boolean endLoop = false;

    final PhotonCamera photonCamera = new PhotonCamera("cam");

    public SpeakerAlign(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.PIDang = new PIDController(3, 0, 0.1);
        this.PIDmove = new PIDController(4.5, 0, 0.1);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        PIDang.reset();
        PIDmove.reset();
        photonCamera.setPipelineIndex(0);
        endLoop = false;
    }

    @Override
    public void execute() {

        var result = photonCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();
            for (PhotonTrackedTarget cur_target : targets) {
                if (cur_target.getFiducialId() == 4 || cur_target.getFiducialId() == 7) {
                    target = cur_target;
                    break;
                }
            }

            if (target.getFiducialId() == 4 || target.getFiducialId() == 7) {

                double d1 = target.getBestCameraToTarget().getX();

                double d = d1 + Constants.Speaker.robotWidth;

                double l = -target.getBestCameraToTarget().getY();
                double AngleL = Math.tanh(l / d);

                PIDang.setSetpoint(0);
                PIDmove.setSetpoint(1.3);

                double rot = PIDang.calculate(AngleL);
                double x = PIDmove.calculate(d1);
                
                if (x > 0) {
                    x = Math.min(x, 1);
                } else if (x < 0){
                    x = Math.max(x, -1);
                } else{
                    x = 0;
                }

                if (Math.abs(x) < 0.1 && Math.abs(rot) < 0.1) {
                    endLoop = true;
                }

                swerve.drive(new Translation2d(x,0), rot, false);

                if (Constants.smartEnable) {
                    SmartDashboard.putBoolean("SpeakerAlign", true);
                    SmartDashboard.putNumber("x", target.getBestCameraToTarget().getX());
                    SmartDashboard.putNumber("y", -target.getBestCameraToTarget().getY());
                    SmartDashboard.putNumber("d1", d1);
                    SmartDashboard.putNumber("AngleL", AngleL);
                    SmartDashboard.putNumber("xcor", x);
                    SmartDashboard.putNumber("rot", rot);

                }
            } else {
                endLoop = true;
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("SpeakerAlign", false);
        }
        swerve.drive(new Translation2d(0, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }

}