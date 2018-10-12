package legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import legacy.ForceofHaskinsHardware;

@Disabled
@TeleOp(name="FoOTeleopRev", group="ForceofHaskins")
public class FoOTeleopRev extends OpMode{

    FoOHardwareREV robot  = new FoOHardwareREV();
    ElapsedTime runTime = new ElapsedTime();

    float movePower = 0;
    float turnPower = 0;
    float latPower = 0;
    float tempadjP = 0;

    float pLim = 1.0f; //power multiplier that acts as a limit
    boolean pLimed = false;
    boolean pLimDown = false;
    float tHold = .4f;
    boolean vServoActive = false;
    boolean hServoActive = false;
    boolean v1Changed = false;
    boolean v1Down = false;
    boolean v2Changed = false;
    boolean v2Down = false;
    boolean hChanged = false;
    boolean hDown = false;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //speed control
        if (gamepad1.y && !pLimDown)
        {
            pLimDown = true;
            if (pLimed)
            {
                pLim = 1.0f;
                pLimed = false;
            }
            else if (!pLimed)
            {
                pLim = .5f;
                pLimed = true;
            }
        }
        else if (!gamepad1.y)
        {
            pLimDown = false;
        }

        //forward/backwards movement
        movePower = gamepad1.left_stick_y * pLim;
        if (Math.abs(gamepad1.left_stick_y) < tHold)
        {
            movePower = 0;
        }

        //side movement
        latPower = -gamepad1.left_stick_x * pLim;
        if (Math.abs(gamepad1.left_stick_x) < tHold)
        {
            latPower = 0;
        }

        //"roTATION!" -Neji Hyuga
        //rotation about the center
        //left forward, right backward
        turnPower = gamepad1.right_stick_x * pLim / 2;
        if (Math.abs(gamepad1.right_stick_x) < tHold)
        {
            turnPower = 0;
        }

        //horz arm adjust
        if (gamepad1.left_trigger > .2)
        {
            tempadjP = .1f;
        }
        else if (gamepad1.left_bumper)
        {
            tempadjP = -.1f;
        }
        else
        {
            tempadjP = 0;
        }

        /*if(gamepad1.x) //reset servos i guess
        {
            robot.vaServoRightB.setPosition(.1);
            robot.vaServoRightT.setPosition(.1);
            robot.vaServoLeftT.setPosition(.87);
            robot.vaServoLeftB.setPosition(.87);
        }*/

        //set the powers
        robot.right.setPower(movePower+turnPower);
        robot.left.setPower(movePower-turnPower);
        robot.center.setPower(latPower);
        robot.horzAdj.setPower(tempadjP);

        //vert arm movement
        while (gamepad1.dpad_up)
        {
            moveArm(robot.verticalarm, 1f);
        }
        while (gamepad1.dpad_down)
        {
            moveArm(robot.verticalarm, -1f);
        }
        robot.verticalarm.setPower(0);

        //horz arm extension
        while (gamepad1.dpad_left)
        {
            moveArm(robot.horizontalArm, -1f); //retract(in theory)
        }
        robot.horizontalArm.setPower(0);
        while (gamepad1.dpad_right)
        {
            moveArm(robot.horizontalArm, 1f); //extend
        }
        robot.horizontalArm.setPower(0);


        //Servo movement
        if (gamepad1.right_bumper && !v1Down)
        {
            if (v1Changed)
            {
                moveCorresServo(1f);
                v1Changed = false;
            }
            else
            {
                moveCorresServo(.5f);
                v1Changed = true;
            }
            v1Down = true;
        }
        else if (!gamepad1.right_bumper)
        {
            v1Down = false;
        }

        if (gamepad1.right_trigger > .2 && !v2Down)
        {
            if (v2Changed)
            {
                moveCorresServo2(1f);
                v2Changed = false;
            }
            else
            {
                moveCorresServo2(.5f);
                v2Changed = true;
            }
            v2Down = true;
        }
        else if (gamepad1.right_trigger <= .2)
        {
            v2Down = false;
        }

        if (gamepad1.x && !hDown)

        {
            if (hChanged)
            {
                robot.haServo.setPosition(1);
                hChanged = false;
            }
            else
            {
                robot.haServo.setPosition(0);
                hChanged = true;
            }
            hDown = true;
        }
        else if (!gamepad1.x)
        {
            hDown = false;
        }

        //adjust horizontal arm vertically
        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        //telemetry.addData("ServoPos", servoPos);
        //telemetry.addData("WristPos", wristAdjust);
        /*
        telemetry.addData("Color R: ", robot.sensorColor.red());
        telemetry.addData("Color G: ", robot.sensorColor.green()); currently not implemented in hardware
        telemetry.addData("Color B: ", robot.sensorColor.blue());
        */
    }

    public void moveArm(DcMotor motor, float power) {
        motor.setPower(power);
        if (motor == robot.verticalarm) {
            vServoActive = true;
            hServoActive = false;
        }
        else if (motor == robot.horizontalArm)
        {
            vServoActive = false;
            hServoActive = true;
        }
    }
    //TODO: Later implement horizontal arm servo detection for all 'corresservo' methods when the horz arm is added in hardware
    public void moveCorresServo(float pos)
    {
        robot.vaServoRightT.setPosition(1-pos);
        robot.vaServoLeftT.setPosition(pos);
    }
    public void moveCorresServo2(float pos)
    {
            robot.vaServoRightB.setPosition(1-pos);
            robot.vaServoLeftB.setPosition(pos);
    }
   /* public void moveMotor(float power)
    {
        robot.horzAdj.setPower(power);
    }*/

}