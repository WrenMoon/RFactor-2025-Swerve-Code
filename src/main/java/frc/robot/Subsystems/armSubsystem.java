package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;



public class armSubsystem extends SubsystemBase {

    final SparkMax motor = new SparkMax(Constants.Arm.ArmID, MotorType.kBrushless); //Creating the SparkMax motor object
  final SparkMaxConfig config = new SparkMaxConfig(); //Creating the config for the SparkMax

  /**
   * The subsystem for the rotswing arm of the robot
   */
  public armSubsystem() {

    config.idleMode(IdleMode.kBrake); //Setting the Config Idle mode to brake
    config.inverted(Constants.Arm.armInvert); //Setting the invert as per the Constants file
    motor.configure(config, null, null); //Applying the configuration to the SparkMax Motor
  }

  @Override
  public void periodic() {
  }

  /**
   * Set the speed for the motor of the arm
   * 
   * @param speed the speed in percentage from 0-1 to set the motor to
   */
  public void setMotor(double speed) {
    motor.set(speed);
  }

  /**
   * Get the position of the arm in encoder ticks
   * 
   * @return encoder position in ticks
   */
  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }

  /**
   * Get the position of the arm in Degrees.
   * 0 degrees is taken as parallel to the floor as per Constants file
   * 
   * @return arm position in degrees
   */
  public double getDegrees() {
    double encoder = motor.getEncoder().getPosition();
    double zeroedEncoder = encoder - Constants.Arm.zeroEncoder;
    return zeroedEncoder * Constants.Arm.degreesPerEncoder;
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

}
