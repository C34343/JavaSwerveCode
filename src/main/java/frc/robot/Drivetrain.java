// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI*4; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.267, 0.267);
  private final Translation2d m_frontRightLocation = new Translation2d(0.267, -0.267);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.267, 0.267);
  private final Translation2d m_backRightLocation = new Translation2d(-0.267, -0.267);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 1, 5.649738110618463);
  private final SwerveModule m_frontRight = new SwerveModule(5, 6, 2, 0.9418667305466683+3.14);
  private final SwerveModule m_backLeft = new SwerveModule(1, 2, 0, 0.16685627318886792);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 3, -0.4638748593756933);
  //-0.09454597736364945

  private final static PigeonIMU m_pigey = new PigeonIMU(11);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics,Rotation2d.fromDegrees(m_pigey.getFusedHeading()));

  public Drivetrain() {
    m_pigey.setFusedHeading(0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,  Rotation2d.fromDegrees(m_pigey.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    //frontright 0.15214992880374822
    //frontleft -0.07802162695054071
    //backleft 0.0
    //backright -0.07353397683834939
    // m_backLeft.printencoder("bl");
    //m_backRight.printencoder("br");
    //m_frontLeft.printencoder("fl");
    //m_frontRight.printencoder("fr");

  }

  public void printEncoders() {
    m_backLeft.printencoder("bl");
     m_backRight.printencoder("br");
     m_frontLeft.printencoder("fl");
     m_frontRight.printencoder("fr");
  }

  public static void resetPidgey(){
    m_pigey.setFusedHeading(0);
  }
  
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
}
