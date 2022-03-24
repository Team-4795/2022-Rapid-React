// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;

/** Add your docs here. */
public class XBoxTrigger extends Button {
    Controller controller;
    int axis = 3;

    public XBoxTrigger(Controller controller, int axis) {
        this.controller = controller;
        this.axis = axis;
    }

    public double getTriggerValue() {
        return controller.getRawAxis(axis);
    }

    @Override
    public boolean get() {
        return controller.getRawAxis(axis) > 0.15;
    }

}