/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.util.PIDConstants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Routine {
        public enum Mechanism {
            SWERVE_TURN,
            SWERVE_DRIVE,
            FLYWHEEL
        }

        public Mechanism ROUTINE = Mechanism.FLYWHEEL;
    }

    public interface Swerve {

        double WIDTH = Units.inchesToMeters(26);
        double LENGTH = Units.inchesToMeters(26);

        SmartNumber MODULE_VELOCITY_DEADBAND =
                new SmartNumber("Swerve/Module velocity deadband (m per s)", 0.02);
        SmartNumber MAX_MODULE_SPEED =
                new SmartNumber("Swerve/Maximum module speed (m per s)", 5.06);
        SmartNumber MAX_MODULE_TURN =
                new SmartNumber("Swerve/Maximum module turn (rad per s)", 6.28);

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 6.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.15);

            SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.44076);
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.0056191);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.00042985);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.00019162);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.36493);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.448);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.16408);
        }

        public interface Motion {

            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-153.984375);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(149.326172);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(71.191406);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(45.351562);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 6.12;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 21.4285714286;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
    }
}
