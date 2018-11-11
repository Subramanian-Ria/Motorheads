package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import legacy.FoOHardware;

/**
 * Created by coolb on 12/20/2017
 */

@Disabled
@Autonomous(name = "Vuforia", group = "insertName")
public class VuforiaTesting extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwarePushbot2 robot   = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //Vuforia Setup
    public static final String TAG = "Vuforia VuMark Sample";
    VuforiaLocalizer vuforia;
    VuforiaTrackables roverTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
    VuforiaTrackable bluePerimeter;
    VuforiaTrackable redPerimeter;
    VuforiaTrackable frontPerimeter;
    VuforiaTrackable backPerimeter;

    VuforiaTrackableDefaultListener blueListener;
    VuforiaTrackableDefaultListener redListener;
    VuforiaTrackableDefaultListener frontListener;
    VuforiaTrackableDefaultListener backListener;

    OpenGLMatrix blueLastKnownLocation;
    OpenGLMatrix redLastKnownLocation;
    OpenGLMatrix frontLastKnownLocation;
    OpenGLMatrix backLastKnownLocation;
    OpenGLMatrix phoneLocation;

    @Override
    public void runOpMode() {

        setupVuforia();

        blueLastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        redLastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        frontLastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        backLastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        roverTrackables.activate();

        //Vuforia
        runtime.reset();
        while(opModeIsActive())
        {
            OpenGLMatrix blueLatestLocation = blueListener.getUpdatedRobotLocation();
            OpenGLMatrix redLatestLocation = blueListener.getUpdatedRobotLocation();
            OpenGLMatrix frontLatestLocation = blueListener.getUpdatedRobotLocation();
            OpenGLMatrix backLatestLocation = blueListener.getUpdatedRobotLocation();

            if(blueLatestLocation != null)
            {
                blueLastKnownLocation = blueLatestLocation;
            }
            else if(redLatestLocation != null)
            {
                redLastKnownLocation = redLatestLocation;
            }
            else if(frontLatestLocation != null)
            {
                frontLastKnownLocation = frontLatestLocation;
            }
            else if(backLatestLocation != null)
            {
                backLastKnownLocation = backLatestLocation;
            }
            telemetry.addData("Tracking " + bluePerimeter.getName(), blueListener.isVisible());
            telemetry.addData("Blue Last Known Location", formatMatrix(blueLastKnownLocation));

            telemetry.addData("Tracking " + redPerimeter.getName(), redListener.isVisible());
            telemetry.addData("Red Last Known Location", formatMatrix(redLastKnownLocation));

            telemetry.addData("Tracking " + frontPerimeter.getName(), frontListener.isVisible());
            telemetry.addData("Front Last Known Location", formatMatrix(frontLastKnownLocation));

            telemetry.addData("Tracking " + backPerimeter.getName(), backListener.isVisible());
            telemetry.addData("Back Last Known Location", formatMatrix(backLastKnownLocation));

            /*glypos = Vuforia(perimeter);
            bluePerimeterPos = Vuforia(bluePerimeter);
            redPerimeterPos = Vuforia(redPerimeter);
            frontPerimeterPos = Vuforia(frontPerimeter);
            backPerimeterPos = Vuforia(backPerimeter);

            telemetry.addData("blue", bluePerimeterPos);
            telemetry.addData("red", redPerimeterPos);
            telemetry.addData("front", frontPerimeterPos);
            telemetry.addData("back", backPerimeterPos);
            telemetry.update();
            */
        }
    }

    public void setupVuforia()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        //Vuforia Message storage do know what glpyh to put it in
        String bluePerimeterPos = "not visible";
        String redPerimeterPos = "not visible";
        String frontPerimeterPos = "not visible";
        String backPerimeterPos = "not visible";

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //More Vuforia Setup
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters VFparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VFparameters.vuforiaLicenseKey = "AXmdhMT/////AAAAGUaKcDAl00pauYqirixLh78YyL/PVKTvRqpancjUCke/xBIuz3p6PjSEmnZ8DShA1mjtzBSG6T9q6JvzM4ARnrGyi4pTQnVUGYgUs8HglThicDhm1xQRvQD3eckc2yjOfC591Bmq1hC4IOGgcMEsOCGPEwgTFKFoZh+c5z/P0Ta6H7S8k1X0Igx9DbLp1/DxU7nsxBKCmOyjsy1kcdB+6zbUiY3CQKqbLYjNEQWs7sStW8+lWtQZ/LA1JBpdXxuZ//68iAjhMVlaojy5nLG3QvcGUZRHSXa3fFyvOcHCYWLfqWzVrUawbj4zVkPQ4LrDdLHPrIviWE+YdgF4mWLvfPOxqzkqMMwmWnnKxMtKqpoG";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        VFparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(VFparameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables roverTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable bluePerimeter = roverTrackables.get(0);
        VuforiaTrackable redPerimeter = roverTrackables.get(1);
        VuforiaTrackable frontPerimeter = roverTrackables.get(2);
        VuforiaTrackable backPerimeter = roverTrackables.get(3);

        bluePerimeter.setName("bluePerimeter"); // can help in debugging; otherwise not necessary
        redPerimeter.setName("redPerimeter");
        frontPerimeter.setName("frontPerimeter");
        backPerimeter.setName("backPerimeter");

        bluePerimeter.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        blueListener = (VuforiaTrackableDefaultListener) bluePerimeter.getListener();
        redListener = (VuforiaTrackableDefaultListener) bluePerimeter.getListener();
        frontListener = (VuforiaTrackableDefaultListener) bluePerimeter.getListener();
        backListener = (VuforiaTrackableDefaultListener) bluePerimeter.getListener();

        blueListener.setPhoneInformation(phoneLocation, VFparameters.cameraDirection);
        redListener.setPhoneInformation(phoneLocation, VFparameters.cameraDirection);
        frontListener.setPhoneInformation(phoneLocation, VFparameters.cameraDirection);
        backListener.setPhoneInformation(phoneLocation, VFparameters.cameraDirection);
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }


    public String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    public String Vuforia(VuforiaTrackable perimeter)
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(perimeter);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

            /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
             * it is perhaps unlikely that you will actually need to act on this pose information, but
             * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)perimeter.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;


            }
            return vuMark.toString();
        }
        else
        {
            telemetry.addData("VuMark", "not visible");
            return "not visible";
        }
    }


}