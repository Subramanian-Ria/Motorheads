//THIS IS A MOTORHEADS PROGRAM

package legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class FoOHardware
{
    /* Public OpMode members. */
    //Initialize everything

    //movement wheels
    public DcMotor flMotor; //front left
    public DcMotor frMotor; //front right
    public DcMotor blMotor; //back left
    public DcMotor brMotor; //back right

    //arms
    public DcMotor verticalarm;
    public DcMotor horizontalarm;
    public Servo haServo;
    public Servo vaServoRightT;
    public Servo vaServoLeftT;
    //public Servo haWrist;
    public DcMotor horzAdj;
    public Servo vaServoRightB;
    public Servo vaServoLeftB;

    //various sensors for autonomous and their servos
    //reading jewel colors
    public ColorSensor sensorColor;
    public Servo colorServo;

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
        flMotor = hwMap.get(DcMotor.class, "flMotor");
        frMotor = hwMap.get(DcMotor.class, "frMotor");
        blMotor = hwMap.get(DcMotor.class, "blMotor");
        brMotor = hwMap.get(DcMotor.class, "brMotor");

        verticalarm = hwMap.get(DcMotor.class, "verticalarm");
        horizontalarm = hwMap.get(DcMotor.class,"horizontalarm");
        horzAdj = hwMap.get(DcMotor.class,"horzAdj");

        // Define Servos
        vaServoRightT = hwMap.get(Servo.class, "vaServoRightT");
        vaServoLeftT = hwMap.get(Servo.class, "vaServoLeftT");
        vaServoLeftB = hwMap.get(Servo.class, "vaServoLeftB");
        vaServoRightB = hwMap.get(Servo.class, "vaServoRightB");
        haServo = hwMap.get(Servo.class, "haServo");
        colorServo = hwMap.get(Servo.class, "colorServo");
        //haWrist = hwMap.get(Servo.class, "haWrist");

        // Define color sensors
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");

        // Defeine stuff for column detecting arm
        /*
        touchArm = hwMap.get(Servo.class, "touchArm");
        touchRed = hwMap.get(DigitalChannel.class, "touchRed");
        touchBlue = hwMap.get(DigitalChannel.class, "touchBlue");
        */

        //Setting Motor Directions
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        flMotor.setDirection(DcMotor.Direction.FORWARD);

        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        verticalarm.setDirection(DcMotor.Direction.FORWARD);
        horizontalarm.setDirection(DcMotor.Direction.FORWARD);
        horzAdj.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        frMotor.setPower(0);
        flMotor.setPower(0);
        brMotor.setPower(0);
        blMotor.setPower(0);

        verticalarm.setPower(0);
        horizontalarm.setPower(0);
        horzAdj.setPower(0);

        //Initialize servo pos
        /*
        vaServoRight.setPosition(.6); //open
        vaServoLeft.setPosition(.7);  //open
        */
        haServo.setPosition(0);
        //haWrist.setPosition(1); //init down
        colorServo.setPosition(0); //init in
        //touchArm.setPosition(1); //init in

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horzAdj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}