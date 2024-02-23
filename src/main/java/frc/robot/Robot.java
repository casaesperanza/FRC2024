// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TROUBLESHOOTING NOTE: MUST COMMENT OUT ANY MOTORS NOT IN USE
//DRIVE MOTORS ARE COMMENTED OUT WHILE TESTING SPINNER
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private static final double INPUT_DEADBAND = 0.13;
  private static final double MAX_INPUT = 1.0;
  private static final double DRIVE_FACTOR = 0.5;
  private static final double UPPER_INTAKE_RATIO = 0.75;

  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(4, MotorType.kBrushed);
  
  private final MotorControllerGroup m_leftMotor = new MotorControllerGroup(m_frontLeftMotor,m_rearLeftMotor);
  private final MotorControllerGroup m_rightMotor = new MotorControllerGroup(m_frontRightMotor,m_rearRightMotor);
  
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  /* PWM DRIVE
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  */

  private final CANSparkMax m_lowerIntakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax m_upperIntakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  //private final CANSparkMax m_hopperMotor = new CANSparkMax(6, MotorType.kBrushless);
  //private final CANSparkMax m_shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_lifterMotor = new CANSparkMax(8, MotorType.kBrushless);

  private final Joystick m_driverController = new Joystick(0);
  private final Joystick m_accessoryController = new Joystick(1);

  public Robot() 
  {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true); 

    m_lowerIntakeMotor.setSmartCurrentLimit(
      75, // stall limit in Amperes kSmartCurrentStallLimit
      75, // free speed limit in Amperes kSmartCurrentFreeLimit
      1000)// RPMs
    ;
    m_upperIntakeMotor.setSmartCurrentLimit(
      75, // stall limit in Amperes kSmartCurrentStallLimit
      75, // free speed limit in Amperes kSmartCurrentFreeLimit
      1000)// RPMs
    ;
    //m_hopperMotor.setSmartCurrentLimit(
    //  20, // stall limit in Amperes kSmartCurrentStallLimit
    //  75, // free speed limit in Amperes kSmartCurrentFreeLimit
    //  2500)// RPMs
    //;
    //m_shooterMotor.setSmartCurrentLimit(
    //  20, // stall limit in Amperes kSmartCurrentStallLimit
    //  75, // free speed limit in Amperes kSmartCurrentFreeLimit
    //  2500)// RPMs
    //;
    m_lifterMotor.setSmartCurrentLimit(
      20, // stall limit in Amperes kSmartCurrentStallLimit
      75, // free speed limit in Amperes kSmartCurrentFreeLimit
      100)// RPMs
    ;
    m_lifterMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    if(Math.abs(m_driverController.getRawAxis(4)) > INPUT_DEADBAND) { // If right Joystick X > DEADBAND, rotate
      m_robotDrive.arcadeDrive(0, -limitInput(m_driverController.getRawAxis(4)));
    }else {
      m_robotDrive.arcadeDrive(-limitInput(m_driverController.getY()), -limitInput(m_driverController.getX()));
    }

    // Spinner (use A or B)
    // (A. COMPETITION CODE) Spin fixed amount on button
    //if(m_stick.getRawButton(1)) {
    // m_spinner.set(.1);
    //} else {
    // m_spinner.set(0);
    //}
    // (B. TESTING ONLY) Spin variably per joystick
    m_lowerIntakeMotor.set(scaleInput(m_accessoryController.getX()));
    m_upperIntakeMotor.set(-UPPER_INTAKE_RATIO * scaleInput(m_accessoryController.getX()));
    //m_hopperMotor.set(-scaleInput(m_accessoryController.getRawAxis(4)));
    //m_shooterMotor.set(-scaleInput(m_accessoryController.getRawAxis(5)));
    m_lifterMotor.set(scaleInput(m_accessoryController.getY()));

    SmartDashboard.putNumber("Front Left Motor Output Current",m_frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Front Right Motor Output Current",m_frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Left Motor Output Current",m_rearLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Right Motor Output Current",m_rearRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Motor Temp",m_lowerIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Output Current",m_lowerIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Upper Motor Temp",m_upperIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Upper Output Current",m_upperIntakeMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Hopper Motor Temp",m_hopperMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Hopper Output Current",m_hopperMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Shooter Motor Temp",m_shooterMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Shooter Output Current",m_shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Lifter Motor Temp",m_lifterMotor.getMotorTemperature());
    SmartDashboard.putNumber("Lifter Output Current",m_lifterMotor.getOutputCurrent());
  }

  protected double scaleInput(double input) {
    if(Math.abs(input) < INPUT_DEADBAND) {
      return 0.0;
    }
    if(Math.abs(input) > MAX_INPUT) {
      return Math.signum(input) * MAX_INPUT;
    }
    return input; //Math.signum(input) * (input * input) / DRIVE_FACTOR;
  }

  protected double limitInput(double input) {
    if(Math.abs(input) < INPUT_DEADBAND) {
      return 0.0;
    }
    if(Math.abs(input) > MAX_INPUT) {
      return Math.signum(input) * MAX_INPUT;
    }
    return input;
  }
}
