// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


public static final int LEFT_DRIVE_MASTER = 1;
public static final int LEFT_DRIVE_FOLLOWER = 2;
public static final int RIGHT_DRIVE_MASTER = 3;
public static final int RIGHT_DRIVE_FOLLOWER = 4;

public static final double DRIVE_SPEED = .5;
public static final double TURN_SPEED = .5;
public static final double RAMP_TIME = .5;

//trajectory constants
public static final double ks = .72;
public static final double kv = 1.36;
public static final double ka = .49;

public static final double kp = 21.113;

public static final double gearRatio = 1/6;

public static final double wheeldistance = .6858;
public static final double wheeldiameter = .1524;
public static final double wheelradius = 3;
public static final double wheelcircumference = wheeldiameter * Math.PI;

public static final double EncoderCPR = 2048;

public static final DifferentialDriveKinematics kDriveKinematics = 
    new DifferentialDriveKinematics(wheeldistance);

public static final double kMaxSpeedMetersPerSecond = 3;
public static final double kMaxAccelerationMetersPerSecondSquared = 3;

public static final double kRamseteB = 2;
public static final double kRamseteZeta = 0.7;

public static final boolean kGyroReversed = true;

}
