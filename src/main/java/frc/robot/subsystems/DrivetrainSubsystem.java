/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private SpeedControllerGroup m_leftMotors;
    private SpeedControllerGroup m_rightMotors;

    private CANSparkMax rearLeftDrive;
    private CANSparkMax rearRightDrive;
    private CANSparkMax frontLeftDrive;
    private CANSparkMax frontRightDrive;

    private DifferentialDrive drive;



    public DrivetrainSubsystem() {
        rearLeftDrive = new CANSparkMax(Constants.rearLeftDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRightDrive = new CANSparkMax(Constants.rearRightDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeftDrive = new CANSparkMax(Constants.frontLeftDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRightDrive = new CANSparkMax(Constants.frontRightDrivePort, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_leftMotors = new SpeedControllerGroup(rearLeftDrive, frontLeftDrive);
        m_rightMotors = new SpeedControllerGroup(rearRightDrive, frontRightDrive);

        drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void arcadeDrive(double power, double turn) {
        drive.arcadeDrive(power, turn);
    }

    public void scaledArcadeDrive(double power, double scaledMaxPower, double turn, double scaledMaxTurn) {
        drive.arcadeDrive(powerScaler(power, scaledMaxPower), powerScaler(turn, scaledMaxTurn));
    }

    public double powerScaler(double input, double maxPower) {
        double output;

        output = scaler(input, -1, 1, -maxPower, maxPower);

        return output;
    }

    private double scaler(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        return outputMin + (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
    }


}
