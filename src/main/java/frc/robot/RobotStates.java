// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum RobotStates {

    IDLE,
    CHARGING,
    SHOOTING,
    ONE_BALL,
    TWO_BALL,
    CLIMBING;

    private static RobotStates rs;

    static {
        rs = RobotStates.IDLE;
    }

    public static RobotStates getState() {
        return rs;
    }

    public static void setState(RobotStates state) {
        rs = state;
    }
}
