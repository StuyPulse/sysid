/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SwerveDriveSysID extends AbstractSysID {

    private final KrakenDriveNeoTurnModuleImpl[] modules;
    private final SysIdRoutine driveRoutine;

    public SwerveDriveSysID() {

        modules =
                new KrakenDriveNeoTurnModuleImpl[] {
                    new KrakenDriveNeoTurnModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER),
                    new KrakenDriveNeoTurnModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER),
                    new KrakenDriveNeoTurnModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER),
                    new KrakenDriveNeoTurnModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER)
                };

        this.driveRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    for (KrakenDriveNeoTurnModuleImpl module : modules) {
                                        module.setMode(true, false);
                                        module.setDriveVoltage(voltage.in(Units.Volts));
                                    }
                                },
                                (log) -> {
                                    for (KrakenDriveNeoTurnModuleImpl module : modules) {
                                        log.motor(module.getID())
                                                .voltage(Units.Volts.of(module.getDriveVoltage()))
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
