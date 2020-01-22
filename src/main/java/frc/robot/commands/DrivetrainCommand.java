/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem m_drive;
    private final OI oi;

    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private NetworkTableEntry maxSpeed = tab.add("Max Speed", 1).getEntry();
    private NetworkTableEntry maxRot = tab.add("Max Turn", 1).getEntry();
    private NetworkTableEntry speed = tab.add("Speed", -127).getEntry();
    private NetworkTableEntry rot = tab.add("Turn", -127).getEntry();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DrivetrainCommand(OI oi, DrivetrainSubsystem subsystem) {
        this.m_drive = subsystem;
        this.oi = oi;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.arcadeDrive(0,0);


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double power = oi.getPower();
        double turn = oi.getTurn();
        double maxPower = maxSpeed.getDouble(1);
        double maxTurn = maxRot.getDouble(1);

        speed.setDouble(power);
        rot.setDouble(turn);

        //SmartDashboard.putNumber("Drive Power", power);
        //SmartDashboard.putNumber("Turn Power", turn);
        //SmartDashboard.putNumber("Max Power", maxPower);
        //SmartDashboard.putNumber("Max Turn", maxTurn);

        m_drive.scaledArcadeDrive(power, maxPower, turn, maxTurn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
