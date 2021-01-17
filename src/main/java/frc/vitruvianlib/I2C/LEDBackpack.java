package frc.vitruvianlib.I2C;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

/**
 * Implementation of an I2C library to utilize an Adafruit .56" 4-digit 7-segment display with HT16K33 I2C Backpack on
 * the roboRIO using WPILib's I2C interface.
 *
 * Code based off of Oracle's I2C guide found here: https://docs.oracle.com/javame/8.2/me-dev-guide/i2c.htm
 */
public class LEDBackpack extends I2C {

    public int[] displaybuffer = new int[10];

    byte[] OSCILLATOR_ON = {0x21};
    byte BRIGHTNESS = (byte) 0xE0;

    static byte HT16K33_BLINK_CMD = (byte) 0x80;
    static byte HT16K33_BLINK_DISPLAYON = (byte) 0x01;

    static byte HT16K33_BLINK_OFF = (byte) 0;
    static byte HT16K33_BLINK_2HZ = (byte) 1;
    static byte HT16K33_BLINK_1HZ = (byte) 2;
    static byte HT16K33_BLINK_HALFHZ = (byte) 3;

    static byte LETTER_J = 0x1E;
    static byte LETTER_A = 0x77;
    static byte LETTER_V = 0x3E;

    public static final byte numbertable[] = {
            0x3F, /* 0 */
            0x06, /* 1 */
            0x5B, /* 2 */
            0x4F, /* 3 */
            0x66, /* 4 */
            0x6D, /* 5 */
            0x7D, /* 6 */
            0x07, /* 7 */
            0x7F, /* 8 */
            0x6F, /* 9 */
            0x77, /* a */
            0x7C, /* b */
            0x39, /* C */
            0x5E, /* d */
            0x79, /* E */
            0x71, /* F */
    };

    public LEDBackpack(Port i2cPort, int address) {
        super(i2cPort, address);
        begin();
    }

    public void begin() {
        ByteBuffer oscOnCmd = ByteBuffer.wrap(OSCILLATOR_ON);
        writeBulk(oscOnCmd, oscOnCmd.remaining());

        setBlinkRate(HT16K33_BLINK_OFF);
        setBrightness(15);
    }

    public void setBrightness(int b) {

        if (b > 15) {
            b = 15;
        } else if (b < 0) {
            b = 0;
        }

        byte[] ea = {(byte) (BRIGHTNESS | b)};

        ByteBuffer brightnessCmd = ByteBuffer.wrap(ea);
        writeBulk(brightnessCmd, brightnessCmd.remaining());
    }

    void setBlinkRate(int b) {

        if (b > 3) {
            b = 0; // turn off if not sure
        } else if (b < 0) {
            b = 0;
        }

        byte[] ea = {(byte) (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1))};

        ByteBuffer blinkRateCmd = ByteBuffer.wrap(ea);
        writeBulk(blinkRateCmd, blinkRateCmd.remaining());
    }

    public void clear() {
        for (int i = 0; i < displaybuffer.length; i++) {
            displaybuffer[i] = 0;
        }
    }

}