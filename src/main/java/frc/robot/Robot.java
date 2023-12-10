/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// importing libraries needed (or not lol)
package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  //private Spark armmotor = new Spark(1);
// setting variable "armmotor" to correspond with the arm motor
  CANSparkMax armmotor = new CANSparkMax(6, MotorType.kBrushless); 

  // intializing the xbox controller
  private final XboxController joy1 = new XboxController(0);
  Joystick xboxcontoller = new Joystick(0);
  //initializing the encoder and calculatingencoder constant
  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12; 

  @Override
  public void robotInit() {
    //m_robotContainer = new RobotContainer();

    // setting the settings for the arm motor
    armmotor.restoreFactoryDefaults();
    armmotor.setInverted(false);
    armmotor.setSmartCurrentLimit(60);

  }

  @Override
  public void autonomousInit() {
    encoder.reset();
  }
 // creating porportional variable and setpoint variable
  final double kP = 0.05;

  double setpoint = 0;

  @Override
  public void autonomousPeriodic() {
    // get xbox command
    if (joy1.getRawButton(1)) {
      // the speed is a percentage
      setpoint = 10; 
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }
    

    // get sensor position
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;
    // output to motors
    armmotor.set(outputSpeed);
  }
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
  }

  @Override
  public void teleopInit() {
    encoder.reset();
  }

  @Override
  public void teleopPeriodic() {
  }


  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}