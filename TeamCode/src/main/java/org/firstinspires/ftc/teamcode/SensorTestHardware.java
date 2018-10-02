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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class SensorTestHardware
{
    /* Public OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    //sensors
    public DistanceSensor sensorDis;
    public ColorSensor sensorCol;
    public DigitalChannel sensorMag;

    //various sensors for autonomous and their servos
    //reading jewel colors  v

    //detecting columns
    /*
    public Servo touchArm;
    public DigitalChannel touchRed; //to use in red autonomous
    public DigitalChannel touchBlue; //to use in blue autonomous
    */

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        //sensors
        sensorDis = hwMap.get(DistanceSensor.class, "sensorDis");
        sensorCol = hwMap.get(ColorSensor.class, "sensorCol");
        sensorMag = hwMap.get(DigitalChannel.class, "sensorMag");
        sensorCol.enableLed(false);

        //Setting Motor Directions
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}