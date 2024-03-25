/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import static com.stuypulse.robot.constants.Settings.Swerve.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Voltage;
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

    private final TalonFX driveMotor;

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;
    private final CANcoder turnAbsoluteEncoder;
    private final AngleController turnController;

    private final SmartBoolean driveSysID;
    private final SmartBoolean turnSysID;

    private double driveCurrent;
    private double turnVoltage;

    private TorqueCurrentFOC currentFOC;

    public SwerveModule(String id, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = 1.0;
        driveMotor.getConfigurator().apply(driveConfig);

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnEncoder = turnMotor.getEncoder();
        turnAbsoluteEncoder = new CANcoder(encoderID);
        turnController =
                new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
                        .setSetpointFilter(new ARateLimit(MAX_MODULE_TURN))
                        .setOutputFilter(x -> -x);

        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        turnMotor.restoreFactoryDefaults();

        driveSysID = new SmartBoolean("Swerve/Modules/Config/Drive SysID Enabled", false);
        turnSysID = new SmartBoolean("Swerve/Modules/Config/Turn SysID Enabled", false);
        
        currentFOC = new TorqueCurrentFOC(0);

        setDriveVoltage(0);
        setDriveCurrent(0);
        setTurnVoltage(0);

        turnMotor.burnFlash();
    }

    /************************************************/

    public String getID() {
        return id;
    }

    public double getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getDriveCurrent() {
        return driveCurrent;
    }

    public double getTurnVoltage() {
        return turnVoltage;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() / Settings.Swerve.Encoder.Drive.VELOCITY_CONVERSION;
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() / Settings.Swerve.Encoder.Drive.POSITION_CONVERSION, getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /************************************************/

    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    public void setDriveCurrent(double current) {
        driveCurrent = current;
        currentFOC.withOutput(current);
        driveMotor.setControl(currentFOC);
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
                setTurnVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getAbsoluteAngle())));
            }

        } else {
            setTurnVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getAbsoluteAngle())));
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", getDriveVoltage());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getDriveVelocity());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Raw Encoder Angle",
                Units.rotationsToDegrees(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Torque Current", driveMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Supply Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Current", driveCurrent);
    }
}
