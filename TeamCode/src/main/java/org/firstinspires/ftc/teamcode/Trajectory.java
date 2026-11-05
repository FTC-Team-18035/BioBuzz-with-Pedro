package org.firstinspires.ftc.teamcode;

public class Trajectory {
    public static double CalculateVelocity(double targetDistance) {
            targetDistance = targetDistance + 0; // TODO: Change this value to offset over or under shooting

            double g = -980.0; // cm/s^2 magnitude of acceleration due to gravity

            double hiveHeight = 152; // cm, target final height in hive

            double m = -0.8; // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE

            double a = (-hiveHeight / (targetDistance * targetDistance)) + (m / targetDistance); // The "a" value in the parabola equation

            double b = ((2.0 * hiveHeight) / targetDistance) - m; // The "b" value in the parabola equation

            double rawLaunchAngle = Math.atan(b); // The launch angle in radians

            double rawLaunchVelocity = ((1.0 / Math.cos(rawLaunchAngle)) * (Math.sqrt(g / (2.0 * a))));  //The launch velocity in cm/s

            return rawLaunchVelocity; 
        }

        public static double CalculateAngle(double targetDistance) {
            targetDistance = targetDistance + 0; // TODO: Change this value to offset over or under shooting

            double g = -980.0; // cm/s^2 magnitude of acceleration due to gravity

            double hiveHeight = 152; // cm, target final height in hive

            double m = -0.8; // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE

            double a = (-hiveHeight / (targetDistance * targetDistance)) + (m / targetDistance); // The "a" value in the parabola equation

            double b = ((2.0 * hiveHeight) / targetDistance) - m; // The "b" value in the parabola equation

            double rawLaunchAngle = Math.toDegrees(Math.atan(b)); // The launch angle in degrees

            return rawLaunchAngle; 
        }

}