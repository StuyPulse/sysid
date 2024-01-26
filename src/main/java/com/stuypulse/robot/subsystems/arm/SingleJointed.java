package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Ports;
import static com.stuypulse.robot.constants.Settings.Arm.SingleJointed.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleJointed extends SubsystemBase {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private double voltage;

    public SingleJointed() {
        motor = new CANSparkMax(Ports.Arm.JOINT_ONE, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(POSITION_CONVERSION);
        encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

        voltage = 0;
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

    @Override
    public void periodic() {
        motor.setVoltage(voltage);
    }
}
