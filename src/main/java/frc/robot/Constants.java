package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;

/**
 * A file to store all the constant values, offset, and other fixed numerical values and details of the robot
 */
public class Constants {
  public static final double ROBOT_MASS = 70;
  public static final Matter CHASSIS = new Matter(new Translation3d(-0.2, 0, Units.inchesToMeters(2)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final boolean smartEnable = true;
  public static final double ControllerDeadband = 0.05;
  public static final boolean VisionOdometry = true;
  public static double MAX_SPEED = 5;

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 100; // seconds
  }

  public static class Intake {
    public static int intakeID = 12;
    public static boolean Invert = false;
    public static final double angleOffset = 40;
    public static final double Kg = 0.03;
    public static final double waitCount = 6;
  }

  public static class Arm {
    public static final int ArmID = 11;
    public static final double maxSpeed = 0.5;
    public static final boolean armInvert = true;
    public static final double MaxSpeed = 0.2;
    public static final double kp = 0.012;
    public static final double kd = 0.0015;
    public static final double Kg = 0.027;
    public static final double degreesPerEncoder = 180 / 22;
    public static final double zeroEncoder = -16.1;

    public static class poses {
      public final double zero = 127.5;
      public final double elevate = 99;
      public final double L1 = 70;
      public final double L4 = 77.5;
      public final double algae = -40;
      public static final double maxPose = 120;
      public static final double minPose = -40;
    }
  }

  public static class Elevator {
    public static final int elevatorLeft = 10;
    public static final int elevatorRight = 9;
    public static final double MaxSpeed = 0.5;
    public static final double MinSpeed = 0.01;
    public static final boolean rightInvert = false;
    public static final boolean leftInvert = true;
    public static final double kp = 0.045;
    public static final double kd = 0.005;
    public static final double Kg = 0.03;

    public static class poses {
      public final double L1 = 0;
      public final double L2 = 12;
      public final double L3 = 43.5;
      public final double L4a = 150 *3/5;
      public final double L4b = 97;
      public final static double maxPose = 97;
      public final static double minPose = 0;
      public final double algae1 = 66 *3/5;
      public final double algae2 = 138 *3/5;
      // public final static double maxPose = 5000;
      // public final static double minPose = -5000;
    }
   }

  public static class CV {
    public static final double kp = 0.2;
    public static final double kd = 0.03;
    public static final double MaxSpeed = 0.5;
    public static final double leftAngle = 8.22;
    public static final double rightAngle = -17.5;
    public static final double middleAngle = 0;
  }

  public static class PosesBlue {
    public static final Pose2d reef1l = new Pose2d(new Translation2d(Meter.of(3.167), Meter.of(4.19)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef1r = new Pose2d(new Translation2d(Meter.of(3.167), Meter.of(3.85)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef2l = new Pose2d(new Translation2d(Meter.of(3.973), Meter.of(5.245)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef2r = new Pose2d(new Translation2d(Meter.of(3.676), Meter.of(5.093)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef3l = new Pose2d(new Translation2d(Meter.of(5.296), Meter.of(5.078)), Rotation2d.fromDegrees(-120));
    public static final Pose2d reef3r = new Pose2d(new Translation2d(Meter.of(4.998), Meter.of(5.253)), Rotation2d.fromDegrees(-120));
    public static final Pose2d reef4l = new Pose2d(new Translation2d(Meter.of(5.811), Meter.of(3.858)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef4r = new Pose2d(new Translation2d(Meter.of(5.811), Meter.of(4.185)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef5l = new Pose2d(new Translation2d(Meter.of(5.005), Meter.of(2.812)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef5r = new Pose2d(new Translation2d(Meter.of(5.288), Meter.of(2.964)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef6l = new Pose2d(new Translation2d(Meter.of(3.683), Meter.of(2.95)), Rotation2d.fromDegrees(60));
    public static final Pose2d reef6r = new Pose2d(new Translation2d(Meter.of(3.973), Meter.of(2.79)), Rotation2d.fromDegrees(60));
    public static final Pose2d stationRight = new Pose2d(new Translation2d(Meter.of(1.213), Meter.of(1.010)), Rotation2d.fromDegrees(55));
    public static final Pose2d stationLeft = new Pose2d(new Translation2d(Meter.of(1.213), Meter.of(7.09)), Rotation2d.fromDegrees(-55));

  }

  public static class PosesRed {
    public static final Pose2d reef1l = new Pose2d(new Translation2d(Meter.of(14.5), Meter.of(4)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef1r = new Pose2d(new Translation2d(Meter.of(14.5), Meter.of(4)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef2l = new Pose2d(new Translation2d(Meter.of(13.8), Meter.of(2.8)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef2r = new Pose2d(new Translation2d(Meter.of(13.8), Meter.of(2.8)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef3l = new Pose2d(new Translation2d(Meter.of(12.3), Meter.of(2.8)), Rotation2d.fromDegrees(60));
    public static final Pose2d reef3r = new Pose2d(new Translation2d(Meter.of(12.3), Meter.of(2.8)), Rotation2d.fromDegrees(60));
    public static final Pose2d reef4l = new Pose2d(new Translation2d(Meter.of(11.6), Meter.of(4)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef4r = new Pose2d(new Translation2d(Meter.of(11.6), Meter.of(4)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef5l = new Pose2d(new Translation2d(Meter.of(12.3), Meter.of(5.3)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef5r = new Pose2d(new Translation2d(Meter.of(12.3), Meter.of(5.3)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef6l = new Pose2d(new Translation2d(Meter.of(13.8), Meter.of(5.3)), Rotation2d.fromDegrees(-120));
    public static final Pose2d reef6r = new Pose2d(new Translation2d(Meter.of(13.8), Meter.of(5.3)), Rotation2d.fromDegrees(-120));
    public static final Pose2d stationLeft = new Pose2d(new Translation2d(Meter.of(16.25), Meter.of(1.010)), Rotation2d.fromDegrees(-55));
    public static final Pose2d stationRight = new Pose2d(new Translation2d(Meter.of(16.25), Meter.of(7.09)), Rotation2d.fromDegrees(55));

  }

}
