// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.ser.std.NumberSerializers.IntLikeSerializer;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final XboxController m_controller = new XboxController(0);
  public static final Drivetrain m_swerve = new Drivetrain();

  int shooterSpeed = 2300;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final PneumaticsControlModule PneumaticsControl = new PneumaticsControlModule(22);
  private final DoubleSolenoid ClimberPneumatic = new DoubleSolenoid(22, PneumaticsModuleType.REVPH, 0, 1);
  private final DoubleSolenoid IntakePneumatic = new DoubleSolenoid(22, PneumaticsModuleType.REVPH, 14, 15);

  private final WPI_VictorSPX ClimberMotorLeft = new WPI_VictorSPX(13);
  private final WPI_VictorSPX ClimberMotorRight = new WPI_VictorSPX(14);

  private final CANSparkMax IntakeMotor = new CANSparkMax(12, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

  boolean intakeToggle = false;

  @Override
  public void robotInit() {
    PneumaticsControl.enableCompressorDigital();
    shooter.shooterInit(); 
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    auto.oneBall();
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    limelight.getLimelight();


    if(m_controller.getRightBumperPressed()){
      intakeToggle = !intakeToggle;
    }
    
    
    SmartDashboard.putBoolean("intake toggle", intakeToggle);
/*
    if(m_controller.getBButtonPressed()){
      shooter.reverseBottomNeckMotor();
    } else {
      shooter.topNeckMotor.set(0.0);
    }
*/
    if(intakeToggle){
      IntakePneumatic.set(Value.kForward);
      IntakeMotor.set(0.6);
      shooter.startBottomNeckMotor();
    }else if(!intakeToggle){
      IntakePneumatic.set(Value.kReverse);
      IntakeMotor.set(0);
    }

    if (m_controller.getYButtonPressed()){
      ClimberPneumatic.set(Value.kForward);
    } else if (m_controller.getAButtonPressed()){
      ClimberPneumatic.set(Value.kReverse);
    }

    if (m_controller.getPOV() == 180){
      ClimberMotorLeft.set(0.3); 
      ClimberMotorRight.set(0.3);
    } else if  (m_controller.getPOV() == 0){
      ClimberMotorLeft.set(-0.3); 
      ClimberMotorRight.set(-0.3);
    } else if (m_controller.getPOV() == 90){
      ClimberMotorRight.set(0.3);
    } else if (m_controller.getPOV() == 270){
      ClimberMotorRight.set(-0.3);
    } else{
      ClimberMotorLeft.set(0); 
      ClimberMotorRight.set(0);
    }

    if (m_controller.getRightTriggerAxis() > 0) {
      shooter.startShooter(shooterSpeed);
    }
    else{
      shooter.stopShooter();
    }

    if (shooter.checkShooterSpeed(shooterSpeed)){
      shooter.runNeckMotors(); 
    }else{
      shooter.stopNeckMotors(intakeToggle);
    }
    // ClimberPneumatic.set(Value.kForward);
    if(m_controller.getRawButtonPressed(8)){
      Drivetrain.resetPidgey();
    }

  }

  @Override
  public void testPeriodic() {
    m_swerve.printEncoders();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    if (m_controller.getLeftBumper()){
      m_swerve.drive(xSpeed * 0.4, ySpeed * 0.4, limelight.lineupShoot() * Drivetrain.kMaxAngularSpeed, true);
    } else {
      m_swerve.drive(xSpeed, ySpeed, rot, true);
    }
  }
}
