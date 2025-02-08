package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenDriveNeoTurnModuleImpl extends SubsystemBase {

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    private final AngleController turnController;

    private final String name;

    private double driveVoltage;
    private double pivotVoltage;

    private SmartBoolean driveSysID;
    private SmartBoolean turnSysID;


    public KrakenDriveNeoTurnModuleImpl(String name, Translation2d location, Rotation2d angleOffset, int driveMotorID, int pivotMotorID, int pivotEncoderID) {
        this.name = name;
        this.angleOffset = angleOffset;

        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(pivotEncoderID, Settings.Swerve.DRIVE_CANBUS);
        
        driveMotor = new TalonFX(driveMotorID, Settings.Swerve.DRIVE_CANBUS);

        driveMotor.getConfigurator().apply(Motors.Swerve.Drive.motorConfig);
        driveMotor.setPosition(0);

        turnController =
                new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        driveSysID = new SmartBoolean("Swerve/Modules/Config/Drive SysID Enabled", false);
        turnSysID = new SmartBoolean("Swerve/Modules/Config/Turn SysID Enabled", false);

        setDriveVoltage(0);
        setTurnVoltage(0);

        driveVoltage = 0;
        pivotVoltage = 0;
    }

    /* GETTERS */

    public String getID() {
        return name;
    }

    public double getTurnVoltage() {
        return pivotMotor.getBusVoltage();
    }

    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurnVelocity() {
        return pivotMotor.get();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    public double getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getPivotVoltage() {
        return pivotMotor.getBusVoltage();
    }

    /* SETTERS */

    public void setDriveVoltage(double voltage) {
        driveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage) {
        pivotVoltage = voltage;
        pivotMotor.setVoltage(voltage);
    }

    public void setMode(boolean driveSysID, boolean turnSysID) {
        this.driveSysID.set(driveSysID);
        this.turnSysID.set(turnSysID);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (DriverStation.isAutonomous()) {

            if (driveSysID.get()) {
                setTurnVoltage(
                        turnController.update(Angle.kZero, Angle.fromRotation2d(getAngle())));
            } else if (turnSysID.get()) {
                setDriveVoltage(0);
            }

        } else {
            setTurnVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getAngle())));
        }

        SmartDashboard.putNumber("Swerve/Drive position " + getName(), getModulePosition().distanceMeters);
    }
}