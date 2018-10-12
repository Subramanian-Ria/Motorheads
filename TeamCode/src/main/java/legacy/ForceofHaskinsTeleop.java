package legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ForceofHaskinsTeleop", group="ForceofHaskins")
@Disabled
public class ForceofHaskinsTeleop extends OpMode{

    ForceofHaskinsHardware robot  = new ForceofHaskinsHardware();
    ElapsedTime runTime = new ElapsedTime();

    double fPower = 0;
    double sPower = 0;
    double turnPower = 0;
    double servoPos = 0.5;
    double vertArmPower = 0;
    double horzArmPower = 0;

    boolean vServoActive = false;
    boolean hServoActive = false;

    float wristAdjust = 1; //set to init position

    int count;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        count = 0;
    }

    @Override
    public void loop() {
        if (count == 0)
        {
            runTime.reset();
        }
        //setting up omni
        if (runTime.seconds() < 90)
        {
            fPower = gamepad1.left_stick_y*.8;
            sPower = gamepad1.left_stick_x*.9;
        }
        else
        {
            fPower = gamepad1.left_stick_y;
            sPower = gamepad1.left_stick_x;
        }


        if (fPower < .05 && fPower > -.05) {
            fPower = 0;
        }
        if (sPower < .05 && sPower > -.05) {
            sPower = 0;
        }

        //turning code
        turnPower = -gamepad1.right_stick_x;
        if (turnPower < .05 && turnPower > -.05)
        {
            turnPower = 0;
        }
        if (turnPower > .05)
        {
            turnPower = .8;
        }
        if (turnPower < -.05)
        {
            turnPower = -.8;
        }

        robot.motorRight.setPower(fPower - turnPower);
        robot.motorLeft.setPower(fPower + turnPower);
        robot.motorCenter.setPower(sPower);

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
            moveArm(robot.horizontalarm, -1f);
        }
        while (gamepad1.dpad_right)
        {
            moveArm(robot.horizontalarm, 1f);
        }
        robot.horizontalarm.setPower(0);

        //Servo movement
        if (gamepad1.left_bumper)
        {
            //robot.vaServo.setPosition(.70);
            moveCorresServo(.8f);
        }
        if (gamepad1.right_bumper)
        {
            //robot.vaServo.setPosition(.30);
            moveCorresServo(0);
        }

        wristAdjust += gamepad1.left_trigger/10 - gamepad1.right_trigger/10;
        if (wristAdjust > 1)
        {
            wristAdjust = 1;
        }
        else if (wristAdjust < 0)
        {
            wristAdjust = 0;
        }
        if (hServoActive)
        {
            robot.haWrist.setPosition(wristAdjust);
        }

        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        telemetry.addData("Left-Power", fPower);
        telemetry.addData("Right-Power", sPower);
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
            robot.vaServoRight.setPosition(pos);
            robot.vaServoLeft.setPosition(1-pos);
        }
        else if (hServoActive)
        {
            robot.haServo.setPosition(pos);
        }
    }
}