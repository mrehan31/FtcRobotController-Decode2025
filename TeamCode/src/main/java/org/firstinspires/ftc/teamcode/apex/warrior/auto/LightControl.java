package org.firstinspires.ftc.teamcode.apex.warrior.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name = "goBILDA RGB Indicator Test", group = "Tests")
public class LightControl extends OpMode {

    // Using ServoImplEx allows us to explicitly set the PWM range
    // to match goBILDA's 500-2500 microsecond spec perfectly.
    private ServoImplEx indicatorLight;

    // A variable to keep track of our current PWM position (0.0 to 1.0)
    private double colorPosition = 0.5;

    @Override
    public void init() {
        // Map the light to the name you used in your config
        indicatorLight = hardwareMap.get(ServoImplEx.class, "indicator_light");

        // Force the PWM range to 500-2500µs so 0.0 and 1.0 map correctly
        indicatorLight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addLine("RGB Indicator Initialized.");
        telemetry.addLine("Use D-Pad Up/Down to change colors.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Increment the position to cycle through the color gradient
        if (gamepad1.dpad_up) {
            colorPosition += 0.005;
        }
        // Decrement the position
        else if (gamepad1.dpad_down) {
            colorPosition -= 0.005;
        }

        // Keep the position strictly within the 0.0 to 1.0 bounds
        colorPosition = Math.max(0.0, Math.min(1.0, colorPosition));

        // Send the PWM signal to the light
        indicatorLight.setPosition(colorPosition);

        // Print the current position so you can note which numbers equal which colors
        telemetry.addData("Current Light Position", "%.3f", colorPosition);
        telemetry.addLine("Take note of the position values when you find a color you like!");
        telemetry.update();
    }
}