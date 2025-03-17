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
    public static final double waitCount = 3;
  }

  public static class Arm {
    public static final int ArmID = 11;
    public static final double maxSpeed = 0.5;
    public static final boolean armInvert = false;
    public static final double MaxPose = -22;
    public static final double MinPose = 0;
    public static final double MaxSpeed = 0.2;
    public static final double kp = 0.007;
    public static final double kd = 0.001;
    public static final double Kg = 0.027;
    public static final double degreesPerEncoder = 180 / 22;
    public static final double zeroEncoder = -15.1;

    public static class poses {
      public final double zero = 123;
      public final double elevate = 87;
      public final double L1 = 70;
      public final double L4 = 98;
      public final double algae = -40;
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
    public static final boolean leftInvert = true;
    public static final double kp = 0.045;
    public static final double kd = 0.005;
    public static final double Kg = 0.01;

    public static class poses {
      public final double L1 = 0;
      public final double L2 = 31;
      public final double L3 = 105;
      public final double L4 = 212;
      public final static double maxPose = 212;
      public final static double minPose = 0;
      public final double algae1 = 66;
      public final double algae2 = 138;
      // public final static double maxPose = 5000;
      // public final static double minPose = -5000;
    }
   }

  public static class CV {
    public static final double kp = 0.15;
    public static final double kd = 0.01;
    public static final double MaxSpeed = 0.35;
    public static final double leftAngle = 9.8;
    public static final double rightAngle = -16.3;
    public static final double middleAngle = -2.5;
  }

  public static class PosesBlue {
    public static final Pose2d reef1 = new Pose2d(new Translation2d(Meter.of(3), Meter.of(4)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef2 = new Pose2d(new Translation2d(Meter.of(3.75), Meter.of(5.3)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef3 = new Pose2d(new Translation2d(Meter.of(5.25), Meter.of(5.3)), Rotation2d.fromDegrees(-120));
    public static final Pose2d reef4 = new Pose2d(new Translation2d(Meter.of(6), Meter.of(4)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef5 = new Pose2d(new Translation2d(Meter.of(5.25), Meter.of(2.8)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef6 = new Pose2d(new Translation2d(Meter.of(3.75), Meter.of(2.8)), Rotation2d.fromDegrees(60));
    public static final Pose2d stationRight = new Pose2d(new Translation2d(Meter.of(1.213), Meter.of(1.010)), Rotation2d.fromDegrees(55));
    public static final Pose2d stationLeft = new Pose2d(new Translation2d(Meter.of(1.213), Meter.of(7)), Rotation2d.fromDegrees(-55));

  }

  public static class PosesRed {
    public static final Pose2d reef1 = new Pose2d(new Translation2d(Meter.of(3.1), Meter.of(3.87)), Rotation2d.fromDegrees(0));
    public static final Pose2d reef2 = new Pose2d(new Translation2d(Meter.of(3.56), Meter.of(5.386)), Rotation2d.fromDegrees(-60));
    public static final Pose2d reef3 = new Pose2d(new Translation2d(Meter.of(5.155), Meter.of(5.546)), Rotation2d.fromDegrees(-120));
    public static final Pose2d reef4 = new Pose2d(new Translation2d(Meter.of(6.143), Meter.of(4.219)), Rotation2d.fromDegrees(180));
    public static final Pose2d reef5 = new Pose2d(new Translation2d(Meter.of(5.474), Meter.of(2.674)), Rotation2d.fromDegrees(120));
    public static final Pose2d reef6 = new Pose2d(new Translation2d(Meter.of(3.819), Meter.of(2.484)), Rotation2d.fromDegrees(60));
    public static final Pose2d stationLeft = new Pose2d(new Translation2d(Meter.of(16.5), Meter.of(1.01)), Rotation2d.fromDegrees(125));
    public static final Pose2d stationRight = new Pose2d(new Translation2d(Meter.of(16.5), Meter.of(7)), Rotation2d.fromDegrees(-125));
  }

}
