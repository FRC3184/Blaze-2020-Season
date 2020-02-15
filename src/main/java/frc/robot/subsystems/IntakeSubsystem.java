/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSPX intake;
    private CANSparkMax arm;

    private DigitalInput lowerLimit;
    private DigitalInput upperLimit;

    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {
        intake = new VictorSPX(Constants.intakeWheel);
        arm = new CANSparkMax(Constants.wrist, CANSparkMaxLowLevel.MotorType.kBrushed);

        lowerLimit = new DigitalInput(Constants.wristLowerLimit);
        upperLimit = new DigitalInput(Constants.wristUpperLimit);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intake (double power) {
        intake.set(ControlMode.PercentOutput, power);
    }

    public void arm (double power) {
        arm.set(-power);
    }
}
