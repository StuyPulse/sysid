/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import static com.stuypulse.robot.constants.Settings.Elevator.*;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

    private final SparkMax motor;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;

    private double voltage;

    public Elevator() {
        motor = new SparkMax(Ports.Elevator.MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();

        motor2 = new SparkMax(Ports.Elevator.MOTOR2, MotorType.kBrushless);
        SparkBaseConfig motor2Config = new SparkMaxConfig().follow(motor).inverted(true);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        voltage = 0;
    }

    public double getVelocity() {
        return encoder.getVelocity() * VELOCITY_CONVERSION;
    }

    public double getPosition() {
        return encoder.getPosition() * POSITION_CONVERSION;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setVoltage(voltage);
    }
}
