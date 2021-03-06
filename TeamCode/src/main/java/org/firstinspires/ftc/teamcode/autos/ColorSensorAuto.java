package org.firstinspires.ftc.teamcode.autos;

/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "ColorSensorAuto", group = "test")
public class ColorSensorAuto extends LinearOpMode {

    public ColorSensor colorSensor;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public DcMotor LeftForward, LeftBack, RightForward, RightBack;
    public RevBlinkinLedDriver blinkblinkboy;
    public double channelDistance;
    Telemetry.Item patternName;


    @Override
    public void runOpMode() {

       // colorSensor = (NormalizedColorSensor)hardwareMap.colorSensor.get("color");
        blinkblinkboy = hardwareMap.get(RevBlinkinLedDriver.class, "blinkblinkboy");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            channelDistance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);


            if(channelDistance <= 2){
                blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else {
                blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            }



           // NormalizedRGBA colors = colorSensor.getNormalizedColors();


/*
            if (colors.red == 255 && colors.green == 255 && colors.blue == 255  && colors.alpha == 0.0) {

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

            }

            telemetry.addData( "red:", colors.red);
            telemetry.addData( "green:", colors.green);
            telemetry.addData( "blue:", colors.blue);
            telemetry.addData( "alpha:", colors.alpha);
            telemetry.update();

*/

        }
    }
}

