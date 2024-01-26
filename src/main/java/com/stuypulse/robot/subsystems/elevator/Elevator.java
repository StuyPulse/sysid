/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import static com.stuypulse.robot.constants.Settings.Elevator.*;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Elevator extends SubsystemBase {

    private final CANSparkMax motor;
    private final CANSparkMax motor2;
    private final RelativeEncoder encoder;

    private double voltage;

    public Elevator() {
        motor = new CANSparkMax(Ports.Elevator.MOTOR, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        encoder = motor.getEncoder();

        motor2 = new CANSparkMax(Ports.Elevator.MOTOR2, MotorType.kBrushless);
        motor2.follow(motor);
        motor2.setInverted(true);

        encoder.setPositionConversionFactor(POSITION_CONVERSION);
        encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

        voltage = 0;

        motor.burnFlash();
        motor2.burnFlash();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setVoltage(voltage);
    }
}
