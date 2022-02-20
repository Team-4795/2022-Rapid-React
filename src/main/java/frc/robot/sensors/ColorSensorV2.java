// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSensorV2 {
  I2C sensor;
  protected final static int PON   = 0b00000001;
  protected final static int AEN   = 0b00000010;
  protected final static int PEN   = 0b00000100;

  public ColorSensorV2() {
    sensor = new I2C(I2C.Port.kMXP, 0x39); //0x39 is the sensor's i2c address
    sensor.write(0x00 | 0x80, 0b00000011); //0b11000000 ... Power on, color sensor on. (page 20 of sensor datasheet)
    sensor.write(0x80 | 0x00, PON | AEN | PEN);
    sensor.write(0x80 | 0x01, (int) (256-10/2.38)); //configures the integration time (time for updating color data)
    sensor.write(0x80 | 0x0E, 0b1111);

    // SmartDashboard.putBoolean("V2 COMMAND WRITE", !trans);
  }

  public Colors getBallColor() {
    Colors ballColor = Colors.Other;

    try {
      if (getProximity() < 20) {
        ballColor = Colors.Other;
      } else if (getBlue() > getRed()) {
        ballColor = Colors.Blue;
      } else if (getRed() > getBlue()) {
        ballColor = Colors.Red;
      }

      SmartDashboard.putNumber("Red v2", getRed());
      SmartDashboard.putNumber("Blue v2", getBlue());
      SmartDashboard.putString("Detected Color v2: ", ballColor == Colors.Red ? "red" : "blue");
    } catch (Exception exception) {
      SmartDashboard.putString("Detected Color v2: ", "error");
    }

    return ballColor;
  }

  public int getVal(int addr) {
    ByteBuffer buf = ByteBuffer.allocate(2);
    boolean trans = sensor.read(0x80 | 0x20 | addr, 2, buf);
    SmartDashboard.putBoolean("READ SUCCESS", !trans);
    buf.order(ByteOrder.LITTLE_ENDIAN);
    // System.out.println(buf);
    return buf.getShort(0);
  }

  public int getRed() {
    //reads the address 0x16, 2 bytes of data, gives it to the buffer
    return getVal(0x16);
  }
  public int getGreen() {
    return getVal(0x18);
  }
  
  public int getBlue() {
    return getVal(0x1A);
  }

  public int getProximity() {
    return getVal(0x1C);
  }

  public int getID() {
    return getVal(0x12);
  }
}