package legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FoOTeleop", group="ForceofHaskins")
@Disabled
public class FoOTeleop extends OpMode{

    FoOHardware robot  = new FoOHardware();
    ElapsedTime runTime = new ElapsedTime();

    double servoPos = 0.5;
    double vertArmPower = 0;
    double horzArmPower = 0;

    boolean vServoActive = false;
    boolean hServoActive = false;

    float wristAdjust = 1; //set to init position

    float frPower = 0; //motor power for future use
    float flPower = 0;
    float brPower = 0;
    float blPower = 0;

    float movePower = 0;
    float turnPower = 0;

    float pLim = .8f; //power multiplier that acts as a limit
    float tHold = .4f;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //forward/backwards movement
        movePower = gamepad1.left_stick_y * pLim;
        if (Math.abs(gamepad1.left_stick_y) < tHold)
        {
            movePower = 0;
        }

        //"roTATION!" -Neji Hyuga
        //rotation about the center
        //left forward, right backward
        turnPower = -gamepad1.right_stick_x * pLim;
        if (Math.abs(gamepad1.right_stick_x) < tHold)
        {
            turnPower = 0;
        }

        flPower = movePower + turnPower;
        blPower = movePower + turnPower;
        frPower = movePower - turnPower;
        brPower = movePower - turnPower;

        //left/right movement (LEGACY)
        /*Only to be used with mechanin wheels!
        else if (Math.abs(gamepad1.left_stick_y) < tHold && Math.abs(gamepad1.left_stick_x) > tHold)
        {
            flPower = (gamepad1.left_stick_x * pLim);
            brPower = (gamepad1.left_stick_x * pLim);

            frPower = -(gamepad1.left_stick_x * pLim);
            blPower = -(gamepad1.left_stick_x * pLim);
        }
        */

        //diagonal movement (LEGACY)
        /*Only to be used with mechanin wheels!
        else if (Math.abs(gamepad1.left_stick_y) > tHold && Math.abs(gamepad1.left_stick_x) > tHold)
        {
            //'northeast' movement
            if (gamepad1.left_stick_y > tHold && gamepad1.left_stick_x > tHold)
            {
                flPower = brPower = ((gamepad1.left_stick_y + gamepad1.left_stick_x)/2) * pLim;
                frPower = blPower = 0;
            }
            //'northwest' movement
            else if (gamepad1.left_stick_y > tHold && gamepad1.left_stick_x < -tHold)
            {
                frPower = blPower = ((gamepad1.left_stick_y + Math.abs(gamepad1.left_stick_x))/2) * pLim;
                flPower = brPower = 0;
            }
            //'southeast' movement
            else if (gamepad1.left_stick_y < -tHold && gamepad1.left_stick_x < -tHold)
            {
                flPower = brPower = 0;
                frPower = blPower = -((gamepad1.left_stick_y + gamepad1.left_stick_x)/2) * pLim;
            }
            //'southwest' movement
            else if (gamepad1.left_stick_y < -tHold && gamepad1.left_stick_x > tHold)
            {
                flPower = brPower = -((Math.abs(gamepad1.left_stick_y) + gamepad1.left_stick_x)/2) * pLim;
                frPower = blPower = 0;
            }
        }
        */

        //set the powers
        robot.frMotor.setPower(frPower);
        robot.flMotor.setPower(flPower);
        robot.blMotor.setPower(blPower);
        robot.brMotor.setPower(brPower);


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

        //horz arm movement
        while (gamepad1.dpad_left)
        {
            moveArm(robot.horizontalarm, -1f); //extend
        }
        robot.horizontalarm.setPower(0);
        while (gamepad1.dpad_right)
        {
            moveArm(robot.horizontalarm, 1f); //extend
        }
        robot.horizontalarm.setPower(0);

        //Servo movement

        if (vServoActive)
        {
            if (gamepad1.left_trigger > .2)
            {
                moveCorresServo2(.8f);
            }
            if (gamepad1.right_trigger > .2)
            {
                moveCorresServo2(.1f);
            }
            if (gamepad1.left_bumper)
            {
                //robot.vaServo.setPosition(.70);
                moveCorresServo(.8f);
            }
            if (gamepad1.right_bumper)
            {
                //robot.vaServo.setPosition(.30);
                moveCorresServo(.1f);
            }
        }
        else if (hServoActive)
        {
           while (gamepad1.left_trigger > .2)
            {
               moveMotor(.3f);
            }
            robot.horzAdj.setPower(0);
            while (gamepad1.right_trigger > .2)
            {
                moveMotor(-.3f);
            }
            robot.horzAdj.setPower(0);
            if(gamepad1.right_bumper)
            {
                robot.haServo.setPosition(0);
            }
            if (gamepad1.left_bumper)
            {
                robot.haServo.setPosition(1);
            }
        }

        //adjust horizontal arm vertically
        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        telemetry.addData("ServoPos", servoPos);
        telemetry.addData("WristPos", wristAdjust);
        /*
        telemetry.addData("Color R: ", robot.sensorColor.red());
        telemetry.addData("Color G: ", robot.sensorColor.green()); currently not implemented in hardware
        telemetry.addData("Color B: ", robot.sensorColor.blue());
        */
    }

    public void moveArm(DcMotor motor, float power)
    {
        motor.setPower(power);
        if (motor == robot.verticalarm)
        {
            vServoActive = true;
            hServoActive = false;
        }
        else if (motor == robot.horizontalarm)
        {
            vServoActive = false;
            hServoActive = true;
        }
    }

    public void moveCorresServo(float pos)
    {
        if(vServoActive)
        {
            //double realPos = pos-.2;
            robot.vaServoRightT.setPosition(pos);
            robot.vaServoLeftT.setPosition(1-pos);
        }
        /*else if (hServoActive)
        {
            robot.haServo.setPosition(pos);
        }*/
    }


    public void moveCorresServo2(float pos)
    {
        if(vServoActive)
        {
            //double realPos = pos-.2;
            robot.vaServoRightB.setPosition(pos);
            robot.vaServoLeftB.setPosition(1-pos);
        }
    }
    public void moveMotor(float power)
    {
        robot.horzAdj.setPower(power);
    }
}