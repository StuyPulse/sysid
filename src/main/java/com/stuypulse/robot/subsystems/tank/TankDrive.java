package com.stuypulse.robot.subsystems.tank;

import static com.stuypulse.robot.constants.Settings.TankDrive.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {

    private final CANSparkMax frontLeft;
    private final CANSparkMax backLeft;
    private final CANSparkMax frontRight;
    private final CANSparkMax backRight;
    
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private double voltage;

    public TankDrive() {
        frontLeft = new CANSparkMax(Ports.TankDrive.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        backLeft = new CANSparkMax(Ports.TankDrive.BACK_LEFT_MOTOR, MotorType.kBrushless);
        frontRight = new CANSparkMax(Ports.TankDrive.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        backRight = new CANSparkMax(Ports.TankDrive.BACK_RIGHT_MOTOR, MotorType.kBrushless);
        
        frontLeft.restoreFactoryDefaults();
        backLeft.restoreFactoryDefaults();
        frontRight.restoreFactoryDefaults();
        backRight.restoreFactoryDefaults();

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        leftEncoder = frontLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        leftEncoder.setPositionConversionFactor(POSITION_CONVERSION);
        leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
        rightEncoder.setPositionConversionFactor(POSITION_CONVERSION);
        rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

        voltage = 0;

        frontLeft.burnFlash();
        backLeft.burnFlash();
        frontRight.burnFlash();
        backRight.burnFlash();
    }

    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public double getPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        frontLeft.setVoltage(voltage);
        frontRight.setVoltage(voltage);
    }
}
