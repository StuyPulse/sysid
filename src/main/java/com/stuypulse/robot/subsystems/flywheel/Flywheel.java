/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.flywheel;

import static com.stuypulse.robot.constants.Settings.Flywheel.*;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Flywheel extends SubsystemBase {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private double voltage;

    public Flywheel() {
        motor = new CANSparkMax(Ports.Flywheel.MOTOR, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(POSITION_CONVERSION);
        encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

        voltage = 0;

        motor.burnFlash();
    }

    public double getVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }

    public double getPosition() {
        return Units.rotationsToRadians(encoder.getPosition());
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setVoltage(voltage);
    }
}
