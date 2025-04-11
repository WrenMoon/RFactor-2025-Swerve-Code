package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {

  final SparkMax motor = new SparkMax(Constants.Intake.intakeID, MotorType.kBrushless); //Creating the SparkMax motor object
  final SparkMaxConfig config = new SparkMaxConfig(); //Creating the config for the SparkMax
  final DigitalInput limitSwitch = new DigitalInput(9); //Create a limit switch on DIO port 9

  /**
   * The subsystem for the roller on the arm of the robot
   */
  public intakeSubsystem() {

    config.idleMode(IdleMode.kBrake); //Setting the Config Idle mode to coast
    config.inverted(Constants.Intake.Invert); //Setting the invert as per the Constants file
    motor.configure(config, null, null);  //Applying the configuration to the SparkMax Motor
  }

  @Override
  public void periodic() {

    //Smartdashboard for debugging
    if (Constants.smartEnable){
      SmartDashboard.putBoolean("Intake/Raw Limit Switch", limitSwitch.get());
    }
  }

  /**
   * Set the speed for the motor of the intake
   * 
   * @param speed the speed in percentage from 0-1 to set the motor to
   */
  public void setMotor(double speed) {
    motor.set(speed);
  }


  /**
   * Command to run the intake at a speed
   * 
   * @param speed DoubleSupplied speed to move the robot at
   * @return Intake Command
   */
  public Command setMotorSupplier(DoubleSupplier speed) {
    return run(() -> {
      motor.set(speed.getAsDouble());
      if (Constants.smartEnable) {
        SmartDashboard.putNumber("Intake/Raw IntakeSpeed", speed.getAsDouble());
      }
    });
  }

  /**
   * Get the position of the intake in encoder ticks
   * 
   * @return encoder position in ticks
   */
  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }
  
  /**
   * Return the state of the limit switch
   * 
   * @return boolean state of the limit switch
   */
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

}
