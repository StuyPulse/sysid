/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Settings.Routine.*;

import com.stuypulse.robot.subsystems.arm.SingleJointedSysID;
import com.stuypulse.robot.subsystems.arm.doublejointed.JointOneSysID;
import com.stuypulse.robot.subsystems.arm.doublejointed.JointTwoSysID;
import com.stuypulse.robot.subsystems.elevator.ElevatorSysID;
import com.stuypulse.robot.subsystems.flywheel.FlywheelSysID;
import com.stuypulse.robot.subsystems.swerve.SwerveDriveSysID;
import com.stuypulse.robot.subsystems.swerve.SwerveTurnSysID;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSysID extends SubsystemBase {

    public static final AbstractSysID instance;

    static {
        switch (ROUTINE) {
            case SWERVE_DRIVE:
                instance = new SwerveDriveSysID();
                break;
            case SWERVE_TURN:
                instance = new SwerveTurnSysID();
                break;
            case FLYWHEEL:
                instance = new FlywheelSysID();
                break;
            case ELEVATOR:
                instance = new ElevatorSysID();
                break;
            case SINGLE_JOINTED_ARM:
                instance = new SingleJointedSysID();
                break;
            case DOUBLE_JOINTED_ARM_JOINT_ONE:
                instance = new JointOneSysID();
                break;
            case DOUBLE_JOINTED_ARM_JOINT_TWO:
                instance = new JointTwoSysID();
                break;
            default:
                instance = null;
                break;
        }
    }

    public static AbstractSysID getInstance() {
        return instance;
    }

    public abstract Command quasistaticForward();

    public abstract Command quasistaticReverse();

    public abstract Command dynamicForward();

    public abstract Command dynamicReverse();
}
