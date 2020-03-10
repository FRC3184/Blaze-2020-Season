package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

public class OI {
    private XboxController controller = new XboxController(0);

    public double getPower() {
        return controller.getY(GenericHID.Hand.kLeft);
    }

    public  double getTurn() {
        return controller.getX(GenericHID.Hand.kRight);
    }

    public boolean armDown() {
        return controller.getAButton();
    }

    public boolean armUp() {
        return  controller.getYButton();
    }

    public boolean intake() {
        if (controller.getTriggerAxis(GenericHID.Hand.kRight) >= .1) {
            return true;
        }

        return false;
    }

    public boolean outtake() {
        return controller.getBButton();
    }

    private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    private NetworkTableEntry maxSpeed = driveTab.addPersistent("Max Speed", 1).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry maxTurn = driveTab.addPersistent("Max Turn", 1).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry speed = driveTab.addPersistent("Speed", -127).withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry turn = driveTab.addPersistent("Turn", -127).withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry reverse = driveTab.addPersistent("Reverse", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

    private NetworkTableEntry armStall = intakeTab.addPersistent("Arm Stall", .25).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry intakeSpeed = intakeTab.addPersistent("Intake Speed", .75).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
    private NetworkTableEntry armPower = intakeTab.addPersistent("Army arm mc arm", .5).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 1)).getEntry();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("OpenSight");
    NetworkTableEntry entry = table.getEntry("led");

    public void turnOnLed() {
        entry.setBoolean(true);
    }

    public double maxSpeed() {
        return maxSpeed.getDouble(1);
    }

    public double maxTurn() {
        return maxTurn.getDouble(1);
    }

    public void putSpeed(double speed) {
        this.speed.setDouble(speed);
    }

    public void putTurn(double turn) {
        this.turn.setDouble(turn);
    }

    public boolean reverse() {
        return reverse.getBoolean(false);
    }

    public double armStall() {
        return armStall.getDouble(.25);
    }

    public double intakeSpeed() {
        return intakeSpeed.getDouble(.75);
    }

    public double armPower() {
        return armPower.getDouble(.5);
    }
}
