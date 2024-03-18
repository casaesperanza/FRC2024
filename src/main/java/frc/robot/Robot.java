// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TROUBLESHOOTING NOTES:
// MUST COMMENT OUT ANY MOTORS/ACCESSORIES NOT IN USE

// Architecture Notes:
// -- Drive System --
// Robot has differential drive using a total of 4 motors, two on each side
// -- Shooter System --
// Robot has an intake system to pull 'Note' inside. Consists of two motors which drive shafts with many rubber wheels
// After 'Note' comes through intake, the hopper helps position it for shooting
// Shooter has one motor which drives two wheels
// -- Lift System --
// Robot has an arm which lifts the full bot. Driven by one motor
// -- Control System -- 
// Robot uses two x-box controllers. One is for driving, one is for controlling accessories

// Requiring Libraries/Support Code
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Defining Robot Components/Variables
public class Robot extends TimedRobot {
  // Drive constants
  private static final double INPUT_DEADBAND = 0.13;
  private static final double MAX_INPUT = 1.0;
  private static final double DRIVE_FACTOR = 0.5;
  private static final double UPPER_INTAKE_RATIO = 0.75;
  private static final double SHOOTER_HIGH_SPEED = 1.0;
  private static final double INTAKE_SPEED = 0.5;
  private static final double LIFTER_SPEED = 0.5;
  private static final double HOPPER_SPEED = 1.0;

  // Auto-aim variables
  private static final double SHOOTER_TARGET_X = 0.0;
  private static final double SHOOTER_TARGET_AREA = 5;
  private static final double AIM_X_VARIANCE = 0.2;
  private static final double AIM_AREA_VARIANCE = 0.2;
  private static final double AIM_SPEED = 0.1;

  // Drive motors - individual motor controllers
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(4, MotorType.kBrushed);

  // Drive motors - left and right side
  private final MotorControllerGroup m_leftMotor = new MotorControllerGroup(m_frontLeftMotor, m_rearLeftMotor);
  private final MotorControllerGroup m_rightMotor = new MotorControllerGroup(m_frontRightMotor, m_rearRightMotor);

  // Create robot drive from left and right
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Accessory motors
  private final CANSparkMax m_bigWheelIntakeMotor = new CANSparkMax(5, MotorType.kBrushed);
  private final CANSparkMax m_smallWheelIntakeMotor = new CANSparkMax(9, MotorType.kBrushed);
  private final CANSparkMax m_hopperMotor = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax m_shooterMotor = new CANSparkMax(7, MotorType.kBrushed);
  private final CANSparkMax m_lifterMotor = new CANSparkMax(8, MotorType.kBrushed);

  // Other robot components - NOTE: Removed hopper switch. Code left for now.
  // DigitalInput m_hopperSwitch = new DigitalInput(0);

  // Controllers
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_accessoryController = new XboxController(1);

  NetworkTable limelight_casaaTable = NetworkTableInstance.getDefault().getTable("limelight_casaa");
  NetworkTableEntry shooterTx = limelight_casaaTable.getEntry("tx");
  NetworkTableEntry shooterTy = limelight_casaaTable.getEntry("ty");
  NetworkTableEntry shooterTa = limelight_casaaTable.getEntry("ta");

  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftMotor); TODO: Why is it
    // commented out? Troubleshooting?
    // SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true);

    m_lifterMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    // Read LimeLight values
    double shooterTargetX = shooterTx.getDouble(0.0);
    double shooterTargetY = shooterTy.getDouble(0.0);
    double shooterTargetArea = shooterTa.getDouble(0.0);

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    if(Math.abs(m_driverController.getRightX()) > INPUT_DEADBAND) { // If right Joystick X > DEADBAND, rotate
      m_robotDrive.arcadeDrive(0, limitInput(m_driverController.getRightX()));
    } else if(m_driverController.getAButtonPressed()) { // Auto-aim at April tag closest to center
      // Correct, aim left to right first
      if(Math.abs(SHOOTER_TARGET_X - shooterTargetX) > AIM_X_VARIANCE) {
        m_robotDrive.arcadeDrive(0, Math.signum(SHOOTER_TARGET_X - shooterTargetX) * AIM_SPEED);
      } else if(Math.abs(SHOOTER_TARGET_AREA - shooterTargetArea) > AIM_AREA_VARIANCE) { // Then correct distance
        m_robotDrive.arcadeDrive(Math.signum(SHOOTER_TARGET_AREA - shooterTargetArea) * AIM_SPEED, 0);
      } else {
        m_robotDrive.arcadeDrive(0, 0);
      }
    } else {
      m_robotDrive.arcadeDrive(limitInput(m_driverController.getLeftY()), limitInput(m_driverController.getLeftX()));
    }

    // Shooter - Right Bumper for Max speed, Right Y axis for scalable speed 
    if(m_accessoryController.getRightBumper()) {
     m_shooterMotor.set(SHOOTER_HIGH_SPEED);
    } else {
     m_shooterMotor.set(-scaleInput(m_accessoryController.getRightY()));
    }

    // Intake - Left Bumper for set speed, Left X axis for scalable speed
    if(m_accessoryController.getLeftBumper()) {
     m_bigWheelIntakeMotor.set(-INTAKE_SPEED);
     m_smallWheelIntakeMotor.set(-UPPER_INTAKE_RATIO * INTAKE_SPEED);
//     if(!m_hopperSwitch.get()) { //  NOTE: removed switch from design. Code left for now. While running the intake and note not tripping switch,
      m_hopperMotor.set(-HOPPER_SPEED); // Run the hopper
//     }
    } else {
     m_bigWheelIntakeMotor.set(-scaleInput(m_accessoryController.getLeftX()));
     m_smallWheelIntakeMotor.set(-scaleInput(UPPER_INTAKE_RATIO*m_accessoryController.getLeftX()));
     
      // Hopper - Left trigger 
      if(Math.abs(m_accessoryController.getLeftTriggerAxis()) > INPUT_DEADBAND) {
        m_hopperMotor.set(- scaleInput(m_accessoryController.getLeftTriggerAxis()));
      } else {
        m_hopperMotor.set(- scaleInput(UPPER_INTAKE_RATIO*m_accessoryController.getLeftX())); // Run the hopper
      }
    }

    //Lifter - Right bumper for set speed, Left Y axis for scalable speed
    if(Math.abs(m_driverController.getLeftTriggerAxis()) > INPUT_DEADBAND) {
     m_lifterMotor.set(scaleInput(m_driverController.getLeftTriggerAxis()));
    }else if(Math.abs(m_driverController.getRightTriggerAxis()) > INPUT_DEADBAND) {
     m_lifterMotor.set(-scaleInput(m_driverController.getRightTriggerAxis()));
    } else {
     m_lifterMotor.set(0);
    }

    // (B. TESTING ONLY) Spin variably per joystick
    //m_hopperMotor.set(-scaleInput(m_accessoryController.getRawAxis(4)));
    //m_shooterMotor.set(-scaleInput(m_accessoryController.getRawAxis(5)));
    //read values periodically

    // Dashboard Outputs
    SmartDashboard.putNumber("Front Left Motor Output Current",m_frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Front Right Motor Output Current",m_frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Left Motor Output Current",m_rearLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Rear Right Motor Output Current",m_rearRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Big Wheel Intake Motor Temp",m_bigWheelIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Big Wheel Intake Output Current",m_bigWheelIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Small Wheel Intake Motor Temp",m_smallWheelIntakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Small Wheel Intake Output Current",m_smallWheelIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hopper Motor Temp",m_hopperMotor.getMotorTemperature());
    SmartDashboard.putNumber("Hopper Output Current",m_hopperMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Motor Temp",m_shooterMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Output Current",m_shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Lifter Motor Temp",m_lifterMotor.getMotorTemperature());
    SmartDashboard.putNumber("Lifter Output Current",m_lifterMotor.getOutputCurrent());

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", shooterTargetX);
    SmartDashboard.putNumber("LimelightY", shooterTargetY);
    SmartDashboard.putNumber("LimelightArea", shooterTargetArea);
  }

  // Helper functions
  protected double scaleInput(double input) {
    if (Math.abs(input) < INPUT_DEADBAND) {
      return 0.0;
    }
    if (Math.abs(input) > MAX_INPUT) {
      return Math.signum(input) * MAX_INPUT;
    }
    return input; // Math.signum(input) * (input * input) / DRIVE_FACTOR;
  }

  protected double limitInput(double input) {
    if (Math.abs(input) < INPUT_DEADBAND) {
      return 0.0;
    }
    if (Math.abs(input) > MAX_INPUT) {
      return Math.signum(input) * MAX_INPUT;
    }
    return input;
  }
}
