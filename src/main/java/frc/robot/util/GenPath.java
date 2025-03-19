package frc.robot.util;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy.FirstCharBasedValidator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmVelocityGains;
import frc.robot.util.ArmPoint;

public class GenPath {

    /**
     * Finds the points along a circular arc for a bend defined by three points.
     *
     * @param p1        Starting point.
     * @param p2        Midpoint where the bend occurs.
     * @param p3        Stopping point.
     * @param radius    Radius of circular arc path.
     * @param numPoints Number of points lying between the arc endpoints.
     * @return A list of ArmPoint points describing a path along the circular arc.
     */
    public static List<Translation2d> getArcPoints(Translation2d p1,
                                                   Translation2d p2,
                                                   Translation2d p3,
                                                   double radius,
                                                   int numPoints) {
        // Translate frame to set p2 as the origin (frame "q")
        Translation2d q1 = p1.minus(p2);
        Translation2d q3 = p3.minus(p2);

        // Inner bend angles from point p2
        Rotation2d thetaB = q3.getAngle().minus(q1.getAngle()); // inner bend angle
        // Divide thetaB by 2 to obtain thetaC (angle from q1 to circle center)
        Rotation2d thetaC = new Rotation2d(thetaB.getRadians() / 2.0);

        // Arc endpoints in frame "q"
        double a = Math.abs(radius / Math.tan(thetaC.getRadians())); // distance from p2 to arc endpoints
        Translation2d qi = fromPolar(a, q1.getAngle());
        Translation2d qf = fromPolar(a, q3.getAngle());

        // Circle center in frame "q"
        double h = Math.abs(radius / Math.sin(thetaC.getRadians())); // distance from p2 to circle center
        Rotation2d centerAngle = q1.getAngle().plus(thetaC);
        Translation2d qc = fromPolar(h, centerAngle);

        // Angles of arc endpoints from circle center
        Rotation2d phiI = (qi.minus(qc)).getAngle();
        Rotation2d phiF = (qf.minus(qc)).getAngle();

        // Interpolate points on the arc between the endpoints
        List<Translation2d> ret = new ArrayList<>();
        for (int i = 0; i < numPoints + 2; i++) {
            double t = (double) i / (numPoints + 1);
            Rotation2d phi = interpolate(phiI, phiF, t);
            // Create a point at the given angle and radius, then translate back by qc and p2
            Translation2d q = fromPolar(radius, phi).plus(qc);
            Translation2d p = q.plus(p2);
            ret.add((p));
        }
        return ret;
    }

    /** adds inflection points for inBend reversal */
    public static List<ArmPoint> generateInflectionPoints(List<ArmPoint> points) {
        for (int i = 0; i < points.size() - 1; i++) {
            if (points.get(i).inBend != points.get(i+1).inBend) {
                Translation2d first = points.get(i).position;
                Translation2d second = points.get(i+1).position;
                double totalLength = ArmConstants.baseStageLength + ArmConstants.secondStageLength;
                double firstToInflection = totalLength - first.getNorm();
                double secondToInflection = totalLength - second.getNorm();
                double fraction = firstToInflection/(firstToInflection + secondToInflection);
                Rotation2d angleDiff = second.getAngle().minus(first.getAngle());
                Rotation2d angle = ((angleDiff).times(fraction)).plus(first.getAngle());
                ArmPoint inflectionPoint1 = new ArmPoint(new Translation2d(totalLength, angle.minus(angleDiff.times(0.1).times(fraction))), points.get(i).inBend);
                ArmPoint inflectionPoint2 = new ArmPoint(new Translation2d(totalLength, angle.plus(angleDiff.times(0.1).times(1-fraction))), points.get(i+1).inBend);
                points.add(i+1, inflectionPoint1);
                points.add(i+2, inflectionPoint2);
                i+=2;
                System.out.println("fraction" + fraction);
                System.out.println("anglediff" + angleDiff.getDegrees());
            }
        }
        int i = 0;
        for (ArmPoint point : points) {
            System.out.println(point.position +""+ i +""+ point.inBend);
            i++;
        }
        return points;
    }

    /**
     * Generates a smooth path through the given list of points by replacing each bend with an arc.
     *
     * @param points    A sequence of ArmPoint points.
     * @param radius    The radius of the arcs.
     * @param numPoints The number of points to generate between each pair of arc endpoints.
     * @return A list of ArmPoint points representing the smooth path.
     */
    public static List<ArmPoint> generateSmoothPath(List<ArmPoint> points,
                                                           double radius,
                                                           int numPoints) {
        for (int i = 0; i < points.size() -1; i++) {
            if (points.get(i).position.getDistance(points.get(i+1).position) < 0.001 && (points.get(i).inBend == points.get(i).inBend)) {
                points.remove(i+1);
                i--;
            }
        }
        List<ArmPoint> ret = new ArrayList<>();
        if (points.isEmpty()) {
            return ret;
        }
        ret.add(points.get(0));
        for (int i = 0; i < points.size() - 2; i++) {
            if ((points.get(i).position.getDistance(points.get(i+1).position) > ArmVelocityGains.arcRadius*2) && (points.get(i+1).position.getDistance(points.get(i+2).position) > ArmVelocityGains.arcRadius*2)) {
                List<ArmPoint> arcPoints = ArmPoint.fromTranslations(getArcPoints(points.get(i).position, points.get(i + 1).position, points.get(i + 2).position, radius, numPoints), points.get(i+1).inBend);
            ret.addAll(arcPoints);
            } else {
                ret.add(points.get(i));
            }
        }
        ret.add(points.get(points.size() - 1));
        return ret;
    }

    public static List<ArmPoint> generateSmoothPath(ArmPoint start, List<ArmPoint> points, ArmPoint end, 
    double radius,
    int numPoints) {
        List<ArmPoint> temp = new ArrayList<>();
        temp.addAll(points);
        temp.add(0, start);
        temp.add(end);
        List<ArmPoint> ret = generateSmoothPath(temp, radius, numPoints);
        return ret;
    }

    /**
     * Helper method to create a Translation2d from polar coordinates.
     *
     * @param r     The radius (distance).
     * @param angle The angle as a Rotation2d.
     * @return A Translation2d point.
     */
    public static Translation2d fromPolar(double r, Rotation2d angle) {
        double x = r * Math.cos(angle.getRadians());
        double y = r * Math.sin(angle.getRadians());
        return new Translation2d(x, y);
    }

    /**
     * Helper method to interpolate between two Rotation2d objects.
     *
     * @param a The starting rotation.
     * @param b The ending rotation.
     * @param t The interpolation parameter (0.0 to 1.0).
     * @return The interpolated Rotation2d.
     */
    public static Rotation2d interpolate(Rotation2d a, Rotation2d b, double t) {
        double start = a.getRadians();
        double end = b.getRadians();
        // Normalize the difference to be within [-pi, pi]
        double diff = end - start;
        while (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        while (diff > Math.PI) {
            diff -= 2 * Math.PI;
        }
        double interpolated = start + t * diff;
        return new Rotation2d(interpolated);
    }

    // Example usage of the above methods.
    public static void main(String[] args) {
        List<ArmPoint> points = new ArrayList<>();
        points.add(new ArmPoint(new Translation2d(1, 0)));
        points.add(new ArmPoint(new Translation2d(2, -1)));
        points.add(new ArmPoint(new Translation2d(3, 2)));
        points.add(new ArmPoint(new Translation2d(2, 2)));
        points.add(new ArmPoint(new Translation2d(4, -1)));
        points.add(new ArmPoint(new Translation2d(1, 1)));
        points.add(new ArmPoint(new Translation2d(2, 4)));
        points.add(new ArmPoint(new Translation2d(2.5, 2.5)));
        points.add(new ArmPoint(new Translation2d(3, 4)));

        double radius = 0.2;
        int numPoints = 5;

        List<ArmPoint> smoothPath = generateSmoothPath(points, radius, numPoints);

        // Print the generated smooth path points.
        for (ArmPoint point : smoothPath) {
            System.out.println("Point: (" + point.position.getX() + ", " + point.position.getY() + ")");
        }
    }
}

