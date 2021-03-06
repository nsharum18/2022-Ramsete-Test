// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();

  XboxController DriverSticks = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
      
    new RunCommand(
      () -> m_drive.arcadeDrive(-DriverSticks.getLeftY(), DriverSticks.getRightX()), m_drive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ks, Constants.kv), 
        Constants.kDriveKinematics, 
        10);

    
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);


    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(

      new Pose2d(0, 0, new Rotation2d(0)),

      List.of(
      
      new Translation2d(1,0)),       

      new Pose2d(3, 1, new Rotation2d(0)),

      config);

    RamseteController m_disabledRamsete = new RamseteController();
    m_disabledRamsete.setEnabled(false);

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(Constants.kp, 0, 0);
    var rightController = new PIDController(Constants.kp, 0, 0);

    RamseteCommand ramseteCommand =
      new RamseteCommand(
        testTrajectory,
        m_drive::getPose,
        m_disabledRamsete,
      //  new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.ks,
          Constants.kv,
          Constants.ka),
          Constants.kDriveKinematics,
          m_drive::getWheelSpeeds,
          leftController,
          rightController,
          (leftVolts, rightVolts) -> {
            m_drive.tankDriveVolts(leftVolts, rightVolts);

            leftMeasurement.setNumber(m_drive.getWheelSpeeds().leftMetersPerSecond);
            leftReference.setNumber(leftController.getSetpoint());

            rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
            rightReference.setNumber(rightController.getSetpoint());
          },
          
          m_drive);


    m_drive.resetOdometry(testTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));

  }
}
