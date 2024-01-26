/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import static com.stuypulse.robot.constants.Settings.Swerve.*;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class SwerveModule extends SubsystemBase {

    private final String id;
    private final Rotation2d angleOffset;

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final Controller driveController;

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;
    private final AngleController turnController;

    private final SmartBoolean driveSysID;
    private final SmartBoolean turnSysID;

    private double driveVoltage;
    private double turnVoltage;

    public SwerveModule(String id, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.angleOffset = angleOffset;

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveEncoder = driveMotor.getEncoder();
        driveController =
                new PIDController(Drive.kP, Drive.kI, Drive.kD)
                        .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
        ;

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnEncoder = turnMotor.getEncoder();
        turnController =
                new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
                        .setSetpointFilter(new ARateLimit(MAX_MODULE_TURN))
                        .setOutputFilter(x -> -x);

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveMotor.restoreFactoryDefaults();

        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        turnMotor.restoreFactoryDefaults();

        driveSysID = new SmartBoolean("Swerve/Modules/Config/Drive SysID Enabled", false);
        turnSysID = new SmartBoolean("Swerve/Modules/Config/Turn SysID Enabled", false);

        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    /************************************************/

    public String getID() {
        return id;
    }

    public double getDriveVoltage() {
        return driveVoltage;
    }

    public double getTurnVoltage() {
        return turnVoltage;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity() * 60);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /************************************************/

    public void setDriveVoltage(double voltage) {
        driveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage) {
        turnVoltage = voltage;
        turnMotor.setVoltage(voltage);
    }

    public void setMode(boolean driveSysID, boolean turnSysID) {
        this.driveSysID.set(driveSysID);
        this.turnSysID.set(turnSysID);
    }

    /************************************************/

    @Override
    public void periodic() {

        if (DriverStation.isAutonomous()) {

            if (driveSysID.get()) {
                setTurnVoltage(
                        turnController.update(Angle.kZero, Angle.fromRotation2d(getAngle())));
            } else if (turnSysID.get()) {
                setDriveVoltage(driveController.update(0, getDriveVelocity()));
            }

        } else {
            setTurnVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getAngle())));
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", getDriveVoltage());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getDriveVelocity());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Raw Encoder Angle",
                Units.rotationsToDegrees(turnEncoder.getPosition()));
    }
}
