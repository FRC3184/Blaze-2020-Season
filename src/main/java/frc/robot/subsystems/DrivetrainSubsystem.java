/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;

    private CANSparkMax rearLeftDrive;
    private CANSparkMax rearRightDrive;
    private CANSparkMax frontLeftDrive;
    private CANSparkMax frontRightDrive;

    private DifferentialDrive drive;

    private AHRS ahrs;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    static final double countsPerRev = 42;
    static final double gearReduction = (50/14)*(48/16);
    static final double wheelCircumfrence = 2*0.1524*Math.PI;
    static final double countsPerMeter = (countsPerRev*gearReduction)/wheelCircumfrence;

    public DrivetrainSubsystem() {
        rearLeftDrive = new CANSparkMax(Constants.rearLeftDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRightDrive = new CANSparkMax(Constants.rearRightDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeftDrive = new CANSparkMax(Constants.frontLeftDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRightDrive = new CANSparkMax(Constants.frontRightDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);

        rearLeftDrive.getEncoder().setVelocityConversionFactor((Math.PI*2*0.152)/60);
        rearRightDrive.getEncoder().setVelocityConversionFactor((Math.PI*2*0.152)/60);

        leftMotors = new SpeedControllerGroup(rearLeftDrive, frontLeftDrive);
        rightMotors = new SpeedControllerGroup(rearRightDrive, frontRightDrive);

        drive = new DifferentialDrive(leftMotors, rightMotors);

        ahrs = new AHRS(SPI.Port.kMXP);
        
        resetEncoders();
        zeroHeading();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(),
                getRightEncoderDistance());
    }

    public void arcadeDrive(double power, double turn) {
        drive.arcadeDrive(power, turn);
    }

    public void scaledArcadeDrive(double power, double scaledMaxPower, double turn, double scaledMaxTurn) {
        drive.arcadeDrive(powerScaler(power, scaledMaxPower), powerScaler(turn, scaledMaxTurn));
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        drive.feed();
    }

    public double powerScaler(double input, double maxPower) {
        double output;

        output = scaler(input, -1, 1, -maxPower, maxPower);

        return output;
    }

    private double scaler(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        return outputMin + (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(rearLeftDrive.getEncoder().getVelocity(), rearRightDrive.getEncoder().getVelocity());
     }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }


    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        rearLeftDrive.getEncoder().setPosition(0);
        rearRightDrive.getEncoder().setPosition(0);
        frontLeftDrive.getEncoder().setPosition(0);
        frontRightDrive.getEncoder().setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }


    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        ahrs.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(ahrs.getAngle(), 360) * (Constants.gyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return ahrs.getRate() * (Constants.gyroReversed ? -1.0 : 1.0);
    }

    public double getLeftEncoderRate(){
        return (rearLeftDrive.getEncoder().getVelocity());
    }

    public double getRightEncoderRate(){
        return (rearRightDrive.getEncoder().getVelocity());
    }

    public double getLeftEncoderTicks(){
        return -(((rearLeftDrive.getEncoder().getPosition())+(frontLeftDrive.getEncoder().getPosition()))/2);
    }

    public double getRightEncoderTicks(){
        return -(((rearRightDrive.getEncoder().getPosition())+(frontRightDrive.getEncoder().getPosition()))/2);
    }

    public double getLeftEncoderDistance(){
        return getLeftEncoderTicks()*countsPerMeter;
    }

    public double getRightEncoderDistance(){
        return getRightEncoderTicks()*countsPerMeter;
    }

}
