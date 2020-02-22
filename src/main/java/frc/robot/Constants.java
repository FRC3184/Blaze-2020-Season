/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static int rearLeftDrivePort = 1;
    public final static int rearRightDrivePort = 3;
    public final static int frontLeftDrivePort = 2;
    public final static int frontRightDrivePort = 4;

    public final static int intakeWheel = 1;
    public final static int wrist = 5;

    public final static int wristLowerLimit = 0;
    public final static int wristUpperLimit = 1;

    public final static boolean gyroReversed = false;

    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.104;
    public static final double kvVoltSecondsPerMeter = 2.83;
    public static final double kaVoltSecondsSquaredPerMeter = 0.204;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.01;

    public static final double kTrackwidthMeters = 0.55316832;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
