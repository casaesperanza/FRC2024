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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  // Drive constants
  private static final double INPUT_DEADBAND = 0.13;
  private static final double MAX_INPUT = 1.0;
  private static final double DRIVE_FACTOR = 0.5;
  private static final double UPPER_INTAKE_RATIO = 0.75;
  private static final double SHOOTER_HIGH_SPEED = 0.8;
  private static final double INTAKE_SPEED = 0.5;

  // Drive motors - individual motor controllers
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(4, MotorType.kBrushed);

  // Drive motors - left and right side
  private final MotorControllerGroup m_leftMotor = new MotorControllerGroup(m_frontLeftMotor,m_rearLeftMotor);
  private final MotorControllerGroup m_rightMotor = new MotorControllerGroup(m_frontRightMotor,m_rearRightMotor);

  // Create robot drive from left and right
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final CANSparkMax m_bigWheelIntakeMotor = new CANSparkMax(5, MotorType.kBrushed);
  private final CANSparkMax m_smallWheelIntakeMotor = new CANSparkMax(9, MotorType.kBrushed);
  //private final CANSparkMax m_hopperMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor = new CANSparkMax(7, MotorType.kBrushed);
  //private final CANSparkMax m_lifterMotor = new CANSparkMax(8, MotorType.kBrushless);

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_accessoryController = new XboxController(1);

  public Robot() 
  {
    //SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    //SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true); 

    //m_bigWheelIntakeMotor.setSmartCurrentLimit(
    //  75, // stall limit in Amperes kSmartCurrentStallLimit
    //  75, // free speed limit in Amperes kSmartCurrentFreeLimit
    //  1000)// RPMs
    //;
    m_bigWheelIntakeMotor.setIdleMode(IdleMode.kBrake);
    //m_smallWheelIntakeMotor.setSmartCurrentLimit(
    //  75, // stall limit in Amperes kSmartCurrentStallLimit
    //  75, // free speed limit in Amperes kSmartCurrentFreeLimit
    //  1000)// RPMs
    //;
    m_smallWheelIntakeMotor.setIdleMode(IdleMode.kBrake);
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
    //m_shooterMotor.setIdleMode(IdleMode.kBrake);
    //m_lifterMotor.setSmartCurrentLimit(
    //  20, // stall limit in Amperes kSmartCurrentStallLimit
    //  75, // free speed limit in Amperes kSmartCurrentFreeLimit
    //  100)// RPMs
    //;
    //m_lifterMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    if(Math.abs(m_driverController.getRightX()) > INPUT_DEADBAND) { // If right Joystick X > DEADBAND, rotate
      m_robotDrive.arcadeDrive(0, -limitInput(m_driverController.getRightX()));
    }else {
      m_robotDrive.arcadeDrive(-limitInput(m_driverController.getLeftY()), -limitInput(m_driverController.getLeftX()));
    }

    // Spinner (use A or B)
    // (A. COMPETITION CODE) Spin fixed amount on button
    if(m_accessoryController.getRightBumper()) {
     m_shooterMotor.set(-SHOOTER_HIGH_SPEED);
    } else {
     m_shooterMotor.set(scaleInput(m_accessoryController.getRightY()));
    }
    if(m_accessoryController.getLeftBumper()) {
     m_bigWheelIntakeMotor.set(-INTAKE_SPEED);
     m_smallWheelIntakeMotor.set(-UPPER_INTAKE_RATIO * INTAKE_SPEED);
    } else {
     m_bigWheelIntakeMotor.set(-scaleInput(m_accessoryController.getLeftX()));
     m_smallWheelIntakeMotor.set(-UPPER_INTAKE_RATIO * scaleInput(m_accessoryController.getLeftX()));
    }
    // (B. TESTING ONLY) Spin variably per joystick
    //m_hopperMotor.set(-scaleInput(m_accessoryController.getRawAxis(4)));
    //m_shooterMotor.set(-scaleInput(m_accessoryController.getRawAxis(5)));
    //m_lifterMotor.set(scaleInput(m_accessoryController.getLeftY()));

    SmartDashboard.putNumber("Front Left Motor Output Current",m_frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Front Right Motor Output Current",m_frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Left Motor Output Current",m_rearLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Right Motor Output Current",m_rearRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Big Wheel Intake Motor Temp",m_bigWheelIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Big Wheel Intake Output Current",m_bigWheelIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Small Wheel Intake Motor Temp",m_smallWheelIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Small Wheel Intake Output Current",m_smallWheelIntakeMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Hopper Motor Temp",m_hopperMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Hopper Output Current",m_hopperMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Motor Temp",m_shooterMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Output Current",m_shooterMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Lifter Motor Temp",m_lifterMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Lifter Output Current",m_lifterMotor.getOutputCurrent());
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
