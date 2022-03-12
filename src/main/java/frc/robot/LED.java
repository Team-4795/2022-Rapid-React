// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class LED {
    private AddressableLED led;
    private AddressableLED led2;
    private AddressableLEDBuffer buffer;
    private int length = 60;

    public LED() {
        led = new AddressableLED(0);
        led2 = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(length);
    }

    public void setRed() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 0);
        }
        led.setData(buffer);
        led2.setData(buffer);
        led.start();
        led2.start();
    }

    public void setBlue() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 255, 0);
        }
        led.setData(buffer);
        led2.setData(buffer);
        led.start();
        led2.start();
    }

    public void setGreen() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 255);
        }
        led.setData(buffer);
        led2.setData(buffer);
        led.start();
        led2.start();
    }

    public void setPurple() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 128, 128, 0);
        }
        led.setData(buffer);
        led2.setData(buffer);
        led.start();
        led2.start();
    }

    public void stop() {
        led.stop();
        led2.stop();
    }
}