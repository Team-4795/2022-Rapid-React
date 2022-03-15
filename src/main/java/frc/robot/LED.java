// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDPreset;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public LED() {
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(27);
        led.setLength(buffer.getLength());
    }

    public void setColor(LEDPreset p) {
        for (int i = (int) Math.round(buffer.getLength() * p.lowPercent); i < Math.round(buffer.getLength() * p.highPercent); i++) buffer.setRGB(i, p.r, p.g, p.b);
        led.setData(buffer);   
    }

    public void clearLED() {
        for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
<<<<<<< Updated upstream
        for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * percent); i--) buffer.setRGB(i, r, g, b);
        led.setData(buffer);
=======
        led.start();
        led.stop();
    }

    public void startLED() {
>>>>>>> Stashed changes
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