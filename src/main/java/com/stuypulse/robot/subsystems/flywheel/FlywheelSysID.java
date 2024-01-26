package com.stuypulse.robot.subsystems.flywheel;

import com.stuypulse.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class FlywheelSysID extends AbstractSysID {

    private final Flywheel flywheel;
    private final SysIdRoutine flywheelRoutine;

    public FlywheelSysID() {
        this.flywheel = new Flywheel();
        this.flywheelRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> voltage) -> {
                                    flywheel.setVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(flywheel.getName())
                                            .voltage(Units.Volts.of(flywheel.getVoltage()))
                                            .angularPosition(Units.Radians.of(flywheel.getPosition()))
                                            .angularVelocity(Units.RadiansPerSecond.of(flywheel.getVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return flywheelRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return flywheelRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return flywheelRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return flywheelRoutine.dynamic(Direction.kReverse);
    }
}
