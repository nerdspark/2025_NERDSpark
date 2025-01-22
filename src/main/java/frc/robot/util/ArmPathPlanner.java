package frc.robot.util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.Arm;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmPathPlanner {
    
    public void armPointSwitcher(Translation2d currentPosition, Translation2d intermediatePoint, Translation2d finalPoint, Arm arm) {

        double distanceToIntermediatePoint = currentPosition.getDistance(intermediatePoint);

        if(distanceToIntermediatePoint < 3) {
            arm.setArmPosition(finalPoint, false);
        }
    }
}
