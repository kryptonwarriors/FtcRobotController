package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColortestbyWizardsexe1 (Blocks to Java)", group = "")
public class Lions_10874 extends LinearOpMode {

    public ColorSensor color1;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() throws InterruptedException{
        int CurrentColor;

        color1 = hardwareMap.get(ColorSensor.class, "color1");

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {

                telemetry.addData("Distance", ((DistanceSensor) color1).getDistance(DistanceUnit.CM));
                telemetry.addData("Red", color1.red());
                telemetry.addData("Green", color1.green());
                telemetry.addData("Blue", color1.blue());
                CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
                telemetry.addData("Hue", JavaUtil.colorToHue(CurrentColor));
                telemetry.addData("Saturation", JavaUtil.colorToSaturation(CurrentColor));
                telemetry.addData("Value", JavaUtil.colorToValue(CurrentColor));
                telemetry.update();

            }
        }
    }
}