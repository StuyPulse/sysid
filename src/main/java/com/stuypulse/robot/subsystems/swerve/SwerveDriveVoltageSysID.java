/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import static com.stuypulse.robot.constants.Settings.Swerve.*;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SwerveDriveVoltageSysID extends AbstractSysID {

    private final SwerveModule[] modules;
    private final SysIdRoutine driveRoutine;

    public SwerveDriveVoltageSysID() {

        modules =
                new SwerveModule[] {
                    new SwerveModule(
                            FrontRight.ID,
                            FrontRight.ABSOLUTE_OFFSET,
                            Ports.Swerve.FrontRight.TURN,
                            Ports.Swerve.FrontRight.DRIVE,
                            Ports.Swerve.FrontRight.ENCODER),
                    new SwerveModule(
                            FrontLeft.ID,
                            FrontLeft.ABSOLUTE_OFFSET,
                            Ports.Swerve.FrontLeft.TURN,
                            Ports.Swerve.FrontLeft.DRIVE,
                            Ports.Swerve.FrontLeft.ENCODER),
                    new SwerveModule(
                            BackLeft.ID,
                            BackLeft.ABSOLUTE_OFFSET,
                            Ports.Swerve.BackLeft.TURN,
                            Ports.Swerve.BackLeft.DRIVE,
                            Ports.Swerve.BackLeft.ENCODER),
                    new SwerveModule(
                            BackRight.ID,
                            BackRight.ABSOLUTE_OFFSET,
                            Ports.Swerve.BackRight.TURN,
                            Ports.Swerve.BackRight.DRIVE,
                            Ports.Swerve.BackRight.ENCODER)
                };

        this.driveRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> voltage) -> {
                                    for (SwerveModule module : modules) {
                                        module.setMode(true, false);
                                        module.setDriveVoltage(voltage.in(Units.Volts));
                                    }
                                },
                                (log) -> {
                                    for (SwerveModule module : modules) {
                                        log.motor(module.getID())
                                                .voltage(Units.Volts.of(module.getDriveVoltage()))
                                                .current(Units.Amps.of(module.getDriveCurrent()))
                                                .linearPosition(
                                                        Units.Meters.of(
                                                                module.getModulePosition()
                                                                        .distanceMeters))
                                                .linearVelocity(
                                                        Units.MetersPerSecond.of(
                                                                module.getModuleState()
                                                                        .speedMetersPerSecond));
                                    }
                                },
                                this));
    }

    public Command quasistaticForward() {
        return driveRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticReverse() {
        return driveRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return driveRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicReverse() {
        return driveRoutine.dynamic(Direction.kReverse);
    }
}
