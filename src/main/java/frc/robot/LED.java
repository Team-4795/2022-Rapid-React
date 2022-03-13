// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public LED() {
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(30);
        led.setLength(buffer.getLength());
    }

    public void setColor(int r, int g, int b, double percent) {
        for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
        for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * percent); i--) buffer.setRGB(i, r, g, b);
        led.setData(buffer);
        led.start();
    }

    // public void setRed() {
    //     for (int i = 0; i < buffer.getLength()/2; i++) {
    //         buffer.setRGB(i, 128, 0, 0);
    //     }
    //     led.setData(buffer);
    //     led.start();
    // }

    // public void setUpperHalfRed() {
    //     for (int i = buffer.getLength()/2; i < buffer.getLength(); i++) {
    //         buffer.setRGB(i, 128, 0, 0);
    //     }
    //     led.setData(buffer);
    // }

    // public void setLowerHalfBlue() {
    //     for (int i = 0; i < buffer.getLength()/2; i++) {
    //         buffer.setRGB(i, 0, 0, 128);
    //     }
    //     led.setData(buffer);
    // }

    // public void startLED() {
    //     setUpperHalfRed();
    //     setLowerHalfBlue();
    //     // setRed();
    //     led.start();
    // }

    // public void setBlue() {
    //     for (int i = 0; i < buffer.getLength(); i++) {
    //         buffer.setRGB(i, 0, 0, 128);
    //     }
    //     for (int i = 0; i < buffer2.getLength(); i++) {
    //         buffer2.setRGB(i, 0, 0, 128);
    //     }
    //     led.setData(buffer);
    //     led2.setData(buffer2);
    //     led.start();
    //     led2.start();
    // }

    // public void setGreen() {
    //     for (int i = 0; i < buffer.getLength(); i++) {
    //         buffer.setRGB(i, 0, 128, 0);
    //     }
    //     for (int i = 0; i < buffer2.getLength(); i++) {
    //         buffer2.setRGB(i, 0, 128, 0);
    //     }
    //     led.setData(buffer);
    //     led2.setData(buffer2);
    //     led.start();
    //     led2.start();
    // }

    // public void stop() {
    //     led.stop();
    // }
}