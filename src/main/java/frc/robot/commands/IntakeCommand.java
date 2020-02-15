/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_subsystem;

    private boolean stallUp = true;

    private OI oi;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(OI oi, IntakeSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        this.oi = oi;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.arm(0);
        m_subsystem.intake(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (oi.armUp()) {
            m_subsystem.arm(oi.armPower());
            stallUp = true;
        } else if (oi.armDown()) {
            m_subsystem.arm(-oi.armPower());
            stallUp = false;
        } else {
            if (stallUp) {
                m_subsystem.arm(oi.armStall());
            } else {
                m_subsystem.arm(-oi.armStall());
            }
        }

        if (oi.intake()) {
            m_subsystem.intake(oi.intakeSpeed());
        } else if (oi.outtake()) {
            m_subsystem.intake(-oi.intakeSpeed());
        } else {
            m_subsystem.intake(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
