/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm.doublejointed;

import static com.stuypulse.robot.constants.Settings.Arm.DoubleJointed.*;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Arm.DoubleJointed.JointOne;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class DoubleJointed extends SubsystemBase {

    private final CANSparkMax jointOne;
    private final RelativeEncoder jointOneEncoder;
    private final AngleController jointOneController;

    private final CANSparkMax jointTwo;
    private final RelativeEncoder jointTwoEncoder;
    private final AngleController jointTwoController;

    private double jointOneVoltage;
    private double jointTwoVoltage;

    private final SmartBoolean jointOneRoutine;
    private final SmartBoolean jointTwoRoutine;

    public DoubleJointed() {
        jointOne = new CANSparkMax(Ports.Arm.JOINT_ONE, MotorType.kBrushless);
        jointOne.restoreFactoryDefaults();
        jointOneEncoder = jointOne.getEncoder();

        jointOneEncoder.setPositionConversionFactor(JointOne.POSITION_CONVERSION);
        jointOneEncoder.setVelocityConversionFactor(JointOne.VELOCITY_CONVERSION);

        jointOneController =
                new AnglePIDController(JointOne.kP, JointOne.kI, JointOne.kD)
                        .add(new AngleArmFeedforward(JointOne.kG.get()));

        jointTwo = new CANSparkMax(Ports.Arm.JOINT_TWO, MotorType.kBrushless);
        jointTwo.restoreFactoryDefaults();
        jointTwoEncoder = jointTwo.getEncoder();

        jointTwoEncoder.setPositionConversionFactor(JointOne.POSITION_CONVERSION);
        jointTwoEncoder.setVelocityConversionFactor(JointOne.VELOCITY_CONVERSION);

        jointTwoController =
                new AnglePIDController(JointTwo.kP, JointTwo.kI, JointTwo.kD)
                        .add(new AngleArmFeedforward(JointTwo.kG.get()));

        jointOneVoltage = 0;
        jointTwoVoltage = 0;

        jointOneRoutine = new SmartBoolean("Double Jointed/Joint One Routine", false);
        jointTwoRoutine = new SmartBoolean("Double Jointed/Joint Two Routine", false);
    }

    public double getJointOneVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(jointOneEncoder.getVelocity());
    }

    public double getJointOnePosition() {
        return Units.rotationsToRadians(jointTwoEncoder.getPosition());
    }

    public double getJointOneVoltage() {
        return jointOneVoltage;
    }

    public void setJointOneVoltage(double voltage) {
        this.jointOneVoltage = voltage;
        jointOne.setVoltage(voltage);
    }

    public double getJointTwoVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(jointOneEncoder.getVelocity());
    }

    public double getJointTwoPosition() {
        return Units.rotationsToRadians(jointTwoEncoder.getPosition());
    }

    public double getJointTwoVoltage() {
        return jointTwoVoltage;
    }

    public void setJointTwoVoltage(double voltage) {
        this.jointTwoVoltage = voltage;
        jointTwo.setVoltage(voltage);
    }

    public void setMode(boolean jointOneRoutine, boolean jointTwoRoutine) {
        this.jointOneRoutine.set(jointOneRoutine);
        this.jointTwoRoutine.set(jointTwoRoutine);
    }

    @Override
    public void periodic() {
        if (jointOneRoutine.get()) {
            jointOne.setVoltage(jointOneVoltage);
            jointTwo.setVoltage(
                    jointTwoController.update(
                            Angle.kZero, Angle.fromRadians(getJointTwoPosition())));
        } else if (jointTwoRoutine.get()) {
            jointOne.setVoltage(
                    jointOneController.update(
                            Angle.k90deg, Angle.fromRadians(getJointOnePosition())));
            jointTwo.setVoltage(jointTwoVoltage);
        }
    }
}
