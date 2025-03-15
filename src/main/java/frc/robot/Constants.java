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
    public static final double waitCount = 2;
  }

  public static class Arm {
    public static final int ArmID = 11;
    public static final double maxSpeed = 0.5;
    public static final boolean armInvert = false;
    public static final double MaxPose = -22;
    public static final double MinPose = 0;
    public static final double MaxSpeed = 0.7;
    public static final double kp = 0.007;
    public static final double kd = 0.001;
    public static final double Kg = 0.027;
    public static final double degreesPerEncoder = 180 / 22;
    public static final double zeroEncoder = -15.1;

    public static class poses {
      public final double zero = 123;
      public final double elevate = 87;
      public final double L1 = 70;
      public final double L4 = 0;
      public final double algae = 0;
      public static final double maxPose = 123;
      public static final double minPose = -90;
    }
  }

  public static class Elevator {
    public static final int elevatorLeft = 10;
    public static final int elevatorRight = 9;
    public static final double MaxSpeed = 1;
    public static final double MinSpeed = 0.01;
    public static final boolean rightInvert = false;
    public static final double kp = 0.045;
    public static final double kd = 0.005;
    public static final double Kg = 0.01;

    public static class poses {
      public final double L1 = 0;
      public final double L2 = 62;
      public final double L3 = 205;
      public final double L4 = 436;
      public final static double maxPose = 436;
      public final static double minPose = 5;
    }
  }

  public static class CV {
    public static final double kp = 0.001;
    public static final double kd = 0.001;
    public static final double MaxSpeed = 0.2;
    public static final double leftAngle = 0;
    public static final double rightAngle = 0;
  }

  public static class reefPosesBlue {
    public static Pose2d reef1 = new Pose2d(new Translation2d(Meter.of(2.792), Meter.of(3.87)), Rotation2d.fromDegrees(0));
    public static Pose2d reef2 = new Pose2d(new Translation2d(Meter.of(2.802), Meter.of(4.18)), Rotation2d.fromDegrees(0));
    public static Pose2d reef3 = new Pose2d(new Translation2d(Meter.of(3.56), Meter.of(5.386)), Rotation2d.fromDegrees(-60));
    public static Pose2d reef4 = new Pose2d(new Translation2d(Meter.of(3.799), Meter.of(5.556)), Rotation2d.fromDegrees(-60));
    public static Pose2d reef5 = new Pose2d(new Translation2d(Meter.of(5.155), Meter.of(5.546)), Rotation2d.fromDegrees(-120));
    public static Pose2d reef6 = new Pose2d(new Translation2d(Meter.of(5.454), Meter.of(5.396)), Rotation2d.fromDegrees(-120));
    public static Pose2d reef7 = new Pose2d(new Translation2d(Meter.of(6.143), Meter.of(4.219)), Rotation2d.fromDegrees(180));
    public static Pose2d reef8 = new Pose2d(new Translation2d(Meter.of(6.133), Meter.of(3.86)), Rotation2d.fromDegrees(180));
    public static Pose2d reef9 = new Pose2d(new Translation2d(Meter.of(5.474), Meter.of(2.674)), Rotation2d.fromDegrees(120));
    public static Pose2d reef10 = new Pose2d(new Translation2d(Meter.of(5.185), Meter.of(2.484)), Rotation2d.fromDegrees(120));
    public static Pose2d reef11 = new Pose2d(new Translation2d(Meter.of(3.819), Meter.of(2.484)), Rotation2d.fromDegrees(60));
    public static Pose2d reef12 = new Pose2d(new Translation2d(Meter.of(3.53), Meter.of(2.664)), Rotation2d.fromDegrees(60));
  }

  public static class reefPosesRed {
    public static Pose2d reef1 = new Pose2d(new Translation2d(Meter.of(15.248), Meter.of(7.603)), Rotation2d.fromDegrees(0));
    public static Pose2d reef2 = new Pose2d(new Translation2d(Meter.of(14.745), Meter.of(3.875)), Rotation2d.fromDegrees(0));
    public static Pose2d reef3 = new Pose2d(new Translation2d(Meter.of(14.026), Meter.of(2.664)), Rotation2d.fromDegrees(-60));
    public static Pose2d reef4 = new Pose2d(new Translation2d(Meter.of(13.738), Meter.of(2.497)), Rotation2d.fromDegrees(-60));
    public static Pose2d reef5 = new Pose2d(new Translation2d(Meter.of(12.335), Meter.of(2.521)), Rotation2d.fromDegrees(-120));
    public static Pose2d reef6 = new Pose2d(new Translation2d(Meter.of(12.084), Meter.of(2.652)), Rotation2d.fromDegrees(-120));
    public static Pose2d reef7 = new Pose2d(new Translation2d(Meter.of(11.4), Meter.of(3.875)), Rotation2d.fromDegrees(180));
    public static Pose2d reef8 = new Pose2d(new Translation2d(Meter.of(11.376), Meter.of(4.163)), Rotation2d.fromDegrees(180));
    public static Pose2d reef9 = new Pose2d(new Translation2d(Meter.of(12.108), Meter.of(5.41)), Rotation2d.fromDegrees(120));
    public static Pose2d reef10 = new Pose2d(new Translation2d(Meter.of(12.395), Meter.of(5.541)), Rotation2d.fromDegrees(120));
    public static Pose2d reef11 = new Pose2d(new Translation2d(Meter.of(13.762), Meter.of(5.541)), Rotation2d.fromDegrees(60));
    public static Pose2d reef12 = new Pose2d(new Translation2d(Meter.of(14.026), Meter.of(5.398)), Rotation2d.fromDegrees(60));
  }

}
