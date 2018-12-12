package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MecanumCraterAuton", group="Mecanum");
//@Disabled

public class MecanumCraterAuton extends LinearOpMode{

    //declare opmode members
    MecanumHardware         robot = new MecanumHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double COUNTS_PER_REV    = 1120 ;// Andymark 40...  TETRIX Motor Encoder = 1440
    static final double DMT       = 3.98;     // Diameter of the wheel
    static final double COUNTS_PER_INCH = COUNTS_PER_REV * Math.PI * DMT;
    static final double DRIVE_SPEED             = 0.6;
    static final double TURN_SPEED              = 0.5;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
    }

}
