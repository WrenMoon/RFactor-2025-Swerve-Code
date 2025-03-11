package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double ROBOT_MASS = 70;
  public static final Matter CHASSIS = new Matter(new Translation3d(-0.2, 0, Units.inchesToMeters(2)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final boolean smartEnable = true;
  public static final double ControllerDeadband = 0.05;

  public static final class DrivebaseConstants {

    public static final double WHEEL_LOCK_TIME = 100; // seconds
  }

  public static class Intake {
    public static int intakeID = 12;
    public static boolean Invert = false;
  }

  public static class Arm {
    public static final int ArmID = 11;
    public static final double maxSpeed = 0.5;
    public static final boolean armInvert = false;
    public static final double MaxPose = -22;
    public static final double MinPose = 0;
    public static final double kp = 0.005;
    public static final double kd = 0.005;
    public static final double holdKp = 0.005;
    public static final double holdKd = 0.005;
    public static final double Kg = 0.1;
    public static final double degreesPerEncoder = 180 / 22;
    public static final double zeroEncoder = -10;
  }

  public static class Elevator {
    public static final int elevatorLeft = 10;
    public static final boolean leftInvert = false;
    public static final int elevatorRight = 9;
    public static final boolean rightInvert = false;
    public static final double kp = 0.1;
    public static final double kd = 0.1;
  }

  public static class Speaker {
    public static double heightOfCam = Units.inchesToMeters(16);
    public static double CamAngle = Units.degreesToRadians(28);
    public static double heightOfAprilTag = 1.5;// 1.44855;
    public static double robotWidth = 0.383;
  }

  public static double MAX_SPEED = 5;
}
