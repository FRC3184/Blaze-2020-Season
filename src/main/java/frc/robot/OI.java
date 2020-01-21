package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private XboxController controller = new XboxController(0);

    public double getPower() {
        return controller.getY(GenericHID.Hand.kLeft);
    }

    public  double getTurn() {
        return controller.getX(GenericHID.Hand.kRight);
    }
}
