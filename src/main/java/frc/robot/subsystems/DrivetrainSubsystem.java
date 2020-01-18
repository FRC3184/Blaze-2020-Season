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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless));

    public DrivetrainSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
