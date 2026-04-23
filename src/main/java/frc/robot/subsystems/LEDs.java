package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import java.util.Random;

public class LEDs {
    private AddressableLED LED;
    private AddressableLEDBuffer LEDBuffer;
    private int bogoCycle;
    private int firstPixelHue;
    private BogoColor[] bogoArray;

    public enum Color {
        RED(0, 255, 128),
        GREEN(60, 255, 128),
        BLUE(130, 255, 128);
        int h, s, v;

        private Color(int h, int s, int v) {
            this.h = h;
            this.s = s;
            this.v = v;
        }
    }

    public enum BogoColor {
        RED1(0, 255, 150, 1),
        RED2(13, 255, 150, 2),
        RED3(26, 255, 150, 3),
        ORANGE1(39, 255, 150, 4),
        ORANGE2(52, 255, 150, 5),
        ORANGE3(65, 255, 150, 6),
        YELLOW1(78, 255, 150, 7),
        YELLOW2(91, 255, 150, 8),
        YELLOW3(104, 255, 150, 9),
        GREEN1(117, 255, 150, 10),
        GREEN2(130, 255, 150, 11),
        GREEN3(143, 255, 150, 12),
        BLUE1(156, 255, 150, 13),
        BLUE2(169, 255, 150, 14);
        int h, s, v, sorting;

        private BogoColor(int h, int s, int v, int sorting) {
            this.h = h;
            this.s = s;
            this.v = v;
            this.sorting = sorting;
        }
    }

    public enum LEDState {
        BOGO,
        ALLIGNED,
        HASPEICE,
        SEETARGET,
        NOPEICE;
    }

    private LEDState currentState;

    public LEDs() {
        LED = new AddressableLED(1);
        LEDBuffer = new AddressableLEDBuffer(720);
        LED.setBitTiming(300, 10000, 1200, 1300);
        LED.setLength(LEDBuffer.getLength());

        currentState = LEDState.NOPEICE;
        LED.setData(LEDBuffer);
        LED.start();

        bogoArray = new BogoColor[] {
            BogoColor.RED1, BogoColor.RED2, BogoColor.RED3,
            BogoColor.ORANGE1, BogoColor.ORANGE2, BogoColor.ORANGE3,
            BogoColor.YELLOW1, BogoColor.YELLOW2, BogoColor.YELLOW3,
            BogoColor.GREEN1, BogoColor.GREEN2, BogoColor.GREEN3,
            BogoColor.BLUE1, BogoColor.BLUE2
        };

        shuffleArray(bogoArray);
        bogoCycle = 0;
    }

    private void shuffleArray(BogoColor[] array) {
        Random random = new Random();
        for (int i = array.length - 1; i > 0; i--) {
            int index = random.nextInt(i + 1);
            BogoColor temp = array[index];
            array[index] = array[i];
            array[i] = temp;
        }
    }

    public void rainbow() {
        for (var i = 0; i < LEDBuffer.getLength(); i++) {
            final var hue = (firstPixelHue + (i * 180 / LEDBuffer.getLength())) % 180;
            LEDBuffer.setHSV(i, hue, 255, 128);
        }

        firstPixelHue = (firstPixelHue + 3) % 180;
        LED.setData(LEDBuffer);
    }

    public void bogoSort() {
        boolean isSorted;
        do {
            isSorted = true;
            for (int i = 0; i < bogoArray.length - 1; i++) {
                if (bogoArray[i].sorting > bogoArray[i + 1].sorting) {
                    isSorted = false;
                    break;
                }
            }

            if (!isSorted) {
                shuffleArray(bogoArray);
            }
        } while (!isSorted);

        // Apply sorted array to LEDs
        for (int i = 0; i < bogoArray.length; i++) {
            setHSV(i * 2, bogoArray[i].h, bogoArray[i].s, bogoArray[i].v);

            if (i < bogoArray.length - 1) {
                setHSV(i * 2 + 1, bogoArray[i].h, bogoArray[i].s, bogoArray[i].v);
            }
        }
    }

    public void workingBogoSort() {
        bogoCycle++;
        if (bogoCycle > 10) {
            bogoSort();
            bogoCycle = 0;
        }
    }

    public void setHSV(int i, int hue, int saturation, int value) {
        LEDBuffer.setHSV(i, hue, saturation, value);
        LED.setData(LEDBuffer);
    }

    public void setHSV(int hue, int saturation, int value) {
        for (int i = 0; i < getBufferLength(); i++) {
            LEDBuffer.setHSV(i, hue, saturation, value);
        }
        LED.setData(LEDBuffer);
    }

    public void setRGB(int i, int red, int green, int blue) {
        LEDBuffer.setRGB(i, red, green, blue);
        LED.setData(LEDBuffer);
    }

    public void setRGB(int red, int green, int blue) {
        for (int i = 0; i < getBufferLength(); i++) {
            LEDBuffer.setRGB(i, red, green, blue);
        }
        LED.setData(LEDBuffer); // Fixed missing data update
    }

    public int getBufferLength() {
        return LEDBuffer.getLength();
    }

    public void sendData() {
        LED.setData(LEDBuffer);
    }

    public void setState(LEDState state) {
        currentState = state;
    }

    public LEDState getState() {
        return currentState;
    }

    public void periodic() {
        switch (currentState) {
            case BOGO:
                bogoCycle++;
                if (bogoCycle > 10) {
                    bogoSort();
                    bogoCycle = 0;
                }
                break;
            case ALLIGNED:
                LEDPattern green = LEDPattern.solid(edu.wpi.first.wpilibj.util.Color.kGreen);
                green.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
                // setHSV(Color.GREEN.h, Color.GREEN.s, Color.GREEN.v);
                break;
            case SEETARGET:
                LEDPattern purple = LEDPattern.solid(edu.wpi.first.wpilibj.util.Color.kWhite);
                purple.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
                break;
            case NOPEICE:
                LEDPattern red = LEDPattern.solid(edu.wpi.first.wpilibj.util.Color.kPurple);
                red.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
                break;
        }
    }
}
