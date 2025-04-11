package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;



public class elevatorSubsystem extends SubsystemBase {

  final SparkMax motorLeft = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless); //Creating the SparkMax motor object for the left motor
  final SparkMax motorRight = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless); //Creating the SparkMax motor object for the right motor
  final SparkMaxConfig leftConfig = new SparkMaxConfig(); //Creating the config for the SparkMax of the left motor
  final SparkMaxConfig rightConfig = new SparkMaxConfig(); //Creating the config for the SparkMax of the right 
  // final DigitalInput limitSwitch = new DigitalInput(8); //Create a limit switch on DIO port 8

  /**
   * The subsystem for the cascade elevator of the robot
   */
  public elevatorSubsystem() {

    leftConfig.idleMode(IdleMode.kBrake); //Setting the Config Idle mode to brake for the left motor
    rightConfig.idleMode(IdleMode.kBrake); //Setting the Config Idle mode to brake for the right motor
    rightConfig.inverted(Constants.Elevator.rightInvert); //Setting the invert for the right motor as per the Constants file
    leftConfig.inverted(Constants.Elevator.leftInvert); //Setting the invert for the left motor as per the Constants file
    // leftConfig.follow(motorRight, false); //Setting the Config of the left motor to follow the right motor

    
    motorLeft.configure(leftConfig, null, null); //Applying the configuration for the left SparkMax Motor
    motorRight.configure(rightConfig, null, null); //Applying the configuration for the right SparkMax Motor

    

  }

  @Override
  public void periodic() {

    if (Constants.smartEnable){
    SmartDashboard.putNumber("Elevator/Raw Left Elevator Encoder", motorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/Raw Right Elevator Encoder", motorRight.getEncoder().getPosition());
    }
  }

  /**
   * Sets the same speed for both motors of the elevator 
   * 
   * @param speed the speed in percentage from 0-1 to set the motor speed
   */
  public void setMotor(double rightSpeed) {

    if (Constants.smartEnable){
      SmartDashboard.putNumber("Elevator/Elevator Raw Speed", rightSpeed);
    }

    motorRight.set(rightSpeed);
    motorLeft.set(rightSpeed);

  }

  /**
   * Get the position of the right motor of the elevator in encoder ticks
   * 
   * @return encoder position in ticks
   */
  public double getEncoder() {
    return motorRight.getEncoder().getPosition();
  }

  public void resetEncoder() {
   motorLeft.getEncoder().setPosition(0);
   motorRight.getEncoder().setPosition(0);
  }
}
