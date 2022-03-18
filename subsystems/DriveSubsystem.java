// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX LMaster = new WPI_TalonFX(Constants.LEFT_DRIVE_MASTER);
  private final WPI_TalonFX LFollower = new WPI_TalonFX(Constants.LEFT_DRIVE_FOLLOWER);
  private final WPI_TalonFX RMaster = new WPI_TalonFX(Constants.LEFT_DRIVE_MASTER);
  private final WPI_TalonFX RFollower = new WPI_TalonFX(Constants.LEFT_DRIVE_MASTER);

  private final DifferentialDrive m_drive = new DifferentialDrive(LMaster, RMaster);

  private final AHRS m_navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;

  Pose2d pose;



  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    LFollower.follow(LMaster);
    LFollower.setInverted(true);
    RFollower.follow(RMaster);
    RFollower.setInverted(false);

    LMaster.setNeutralMode(NeutralMode.Brake);
    LFollower.setNeutralMode(NeutralMode.Brake);
    RMaster.setNeutralMode(NeutralMode.Brake);
    RFollower.setNeutralMode(NeutralMode.Brake);

    LMaster.setInverted(true);
    RMaster.setInverted(false);

    LMaster.configOpenloopRamp(Constants.RAMP_TIME);
    RMaster.configOpenloopRamp(Constants.RAMP_TIME);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  m_odometry.update(m_navx.getRotation2d(), getLMasterEncoder(), getRMasterEncoder());

  }


  public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(
      getLMasterRate() * Constants.gearRatio * 10 / Constants.EncoderCPR * Constants.wheelcircumference,
      getRMasterRate() * Constants.gearRatio * 10 / Constants.EncoderCPR * Constants.wheelcircumference
      );

  }

  public void resetOdometry(Pose2d pose) {

    resetEncoders();
    m_odometry.resetPosition(pose, m_navx.getRotation2d());

  }

  public void arcadeDrive(double fwd, double rot) {

    m_drive.arcadeDrive(fwd * Constants.DRIVE_SPEED, rot * Constants.TURN_SPEED);

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    LMaster.setVoltage(leftVolts);
    RMaster.setVoltage(rightVolts);
    m_drive.feed();

  }

  public void resetEncoders() {

    LMaster.setSelectedSensorPosition(0);
    RMaster.setSelectedSensorPosition(0);

  }

  public double getLMasterEncoder() {

    return LMaster.getSelectedSensorPosition() / Constants.EncoderCPR * Constants.gearRatio * Constants.wheelcircumference;
    
  }

  public double getRMasterEncoder() {

    return RMaster.getSelectedSensorPosition() / Constants.EncoderCPR * Constants.gearRatio * Constants.wheelcircumference;

  }

  public double getLMasterRate() {

    return LMaster.getSelectedSensorVelocity();

  }

  public double getRMasterRate() {

    return RMaster.getSelectedSensorVelocity();

  }

  public void zeroHeading() {

    m_navx.reset();

  }

  public double getHeading() {

    return m_navx.getRotation2d().getDegrees();

  }

  public double getTurnRate() {

    return -m_navx.getRate();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
