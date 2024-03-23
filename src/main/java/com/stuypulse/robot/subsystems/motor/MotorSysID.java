package com.stuypulse.robot.subsystems.motor;

import com.stuypulse.robot.subsystems.AbstractSysID;
import com.stuypulse.utils.SysIdRoutine;
import com.stuypulse.utils.SysIdRoutine.Direction;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class MotorSysID extends AbstractSysID {

    private final KrakenMotor motor;
    private final SysIdRoutine motorRoutine;

    public MotorSysID() {
        this.motor = new KrakenMotor();
        this.motorRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(), 
                new SysIdRoutine.Mechanism(
                    (Measure<Current> current) -> {
                        motor.setCurrent(current.in((Units.Amps)));
                    }, 
                    (log) -> {
                        log.motor(motor.getName())
                            .voltage(Units.Volts.of(motor.getVoltage()))
                            .current(Units.Amps.of(motor.getCurrent()))
                            .angularPosition(
                                Units.Rotations.of(
                                    motor.getPosition()))
                            .angularVelocity(
                                Units.RotationsPerSecond.of(
                                    motor.getVelocity()));
                    }, 
                    this));
    }

    public Command quasistaticForward() {
        return motorRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticReverse() {
        return motorRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return motorRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicReverse() {
        return motorRoutine.dynamic(Direction.kReverse);
    }
    
}
