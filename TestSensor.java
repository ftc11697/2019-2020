/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * This is an example LinearOpMode that shows how to use
 * a REV Robotics Touch Sensor.
 *
 * It assumes that the touch sensor is configured with a name of "sensor_digital".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@TeleOp(name = "TestSensor", group = "11697")
//@Disabled
public class TestSensor extends LinearOpMode {
    /**
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     *
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */
    Hardware robot = new Hardware();
    DigitalChannel cantTouchThis;  // Hardware Device Object

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Name sensors
        cantTouchThis = hardwareMap.get(DigitalChannel.class, "cantTouchThis");
        // set the digital channel to input
        cantTouchThis.setMode(DigitalChannel.Mode.INPUT);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        int testLoop = 0;
        while (opModeIsActive()) {
            //testing purposes
            /*boolean sensorState= cantTouchThis.getState();
            testLoop++;
            telemetry.addData(cantTouchThis.getMode()+ " state" + testLoop + ":", sensorState); */

//            while(cantTouchThis.getState() == true) {
//                telemetry.addData("Touch Sensor", "Is Pressed");
//                robot.pinchyBoi.setPosition(0.48);
//                //go to -1475
////                robot.verticalMotor.setTargetPosition(-1475);
////                robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                robot.verticalMotor.setPower(.2);
//                //pinch the block
//            }

            if(cantTouchThis.getState() == true) {
                telemetry.addData("", "pressed");
                robot.pinchyBoi.setPosition(0.48);
                robot.verticalMotor.setTargetPosition(-1475);
                robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalMotor.setPower(.2);

            } else {
                telemetry.addData("", "", "not pressed ");
                robot.pinchyBoi.setPosition(0.12);
            }

            telemetry.update();
        }
    }
}