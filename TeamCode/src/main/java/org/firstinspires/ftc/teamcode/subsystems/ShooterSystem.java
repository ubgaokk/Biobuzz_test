package org.firstinspires.ftc.teamcode.subsystems;

import java.util.*;

public class ShooterSystem {
    public static class ShooterPoint { // struct
        public final double distance; // inches
        public final double rpm;
        public final double angle;    // servo position or degrees

        public ShooterPoint(double distance, double rpm, double angle) { // constructor
            // distance from goal, rpm of flywheel, and hood angle
            this.distance = distance;
            this.rpm = rpm;
            this.angle = angle;
        }
    }

    public static class ShooterLookupTable { // lookup table class

        private final List<ShooterPoint> table = new ArrayList<>(); // mutable lists!

        public ShooterLookupTable(List<ShooterPoint> points) {
            table.addAll(points);
            table.sort(Comparator.comparingDouble(p -> p.distance));
        }

        public ShooterPoint get(double distance, double batteryVoltage) {
            ShooterPoint base = interpolate(distance);

            return new ShooterPoint(distance, base.rpm, base.angle);
        }

        private ShooterPoint interpolate(double distance) { // interpolation
            if (distance <= table.get(0).distance) return table.get(0); // out of bounds small
            if (distance >= table.get(table.size() - 1).distance) // out of bounds large
                return table.get(table.size() - 1);

            for (int i = 0; i < table.size() - 1; i++) { // can replace w/ binary search
                ShooterPoint p0 = table.get(i);
                ShooterPoint p1 = table.get(i + 1);

                if (distance >= p0.distance && distance <= p1.distance) {
                    double t = (distance - p0.distance) /
                            (p1.distance - p0.distance); // distance ratio for interpol calc

                    return new ShooterPoint(
                            distance,
                            p0.rpm + (p1.rpm - p0.rpm) * t, // interpolated value
                            p0.angle + (p1.angle - p0.angle) * t
                    );
                }
            }
            return table.get(0); // default (should not occur unless error)
        }
    }

    public static ShooterLookupTable createDefaultTable() {
        return new ShooterLookupTable(List.of( // TUNE DATA
                new ShooterPoint(2, 2800, 0.32),
                new ShooterPoint(72, 3950, 0.52)
        ));
    }
}