package com.stuypulse.robot.subsystems.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMotor extends SubsystemBase {
    
    private final TalonFX motor;

    private double motorVoltage;
    private double motorCurrent;

    private final TorqueCurrentFOC currentFOC;
    private final VoltageOut currentVoltage;

    public KrakenMotor() {
        motor = new TalonFX(Ports.Motor.MOTOR);
        
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motor.getConfigurator().apply(motorConfig);

        currentFOC = new TorqueCurrentFOC(0);
        currentVoltage = new VoltageOut(0);
    }

    /************************************************/

    public double getVoltage() {
        return motorVoltage;
    }

    public double getCurrent() {
        return motorCurrent;
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /************************************************/

    public void setVoltage(double voltage) {
        motorVoltage = voltage;
        currentVoltage.withOutput(voltage);
        motor.setControl(currentVoltage);
    }

    public void setCurrent(double current) {
        motorCurrent = current;
        currentFOC.withOutput(current);
        motor.setControl(currentFOC);
    }
}
