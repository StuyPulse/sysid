package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*
 * File containing all the robot constants
 *
 * Such constants include:
 *  - Length and width
 *  - Max and min heights for a elevator
 * 
 * This file is intended to make it easier to double check constants with engineering since everything is in one file
 */
public interface Constants {

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(18.75);
        double LENGTH = Units.inchesToMeters(18.75);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 5.36;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
            }
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.149902);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.270752);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.113037);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.441162);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
    }

    public interface Elevator {
        double MIN_HEIGHT_METERS = Units.inchesToMeters(18 + 1.0/16.0); // FROM THE FLOOR TO TOP OF CARRAIGE
        double MAX_HEIGHT_METERS = Units.inchesToMeters(90.5); // FROM THE FLOOR TO TOP OF CARRAIGE

        double DRUM_RADIUS_METERS = (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEARING) / 2 / Math.PI;

        public interface Encoders {
            double GEARING = 4.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (6 + 2.0 / 24) * GEARING; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }
}
