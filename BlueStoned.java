/*
https://github.com/Rambotics/FTC-2016-2017-v2.4-pc/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
https://github.com/pmtischler/ftc_app/tree/master/SharedCode/src/main/java/com/github/pmtischler

Code Folding Expand/Collapse ALL => Control || Shift || +/-
Last Edit Location => Control + Shift + BackSpace
Add Bookmark => Control + F11 + number
Find Bookmark => Control + number
Show Bookmarks => Shift + F11
Jump to Declaration => Control + B


*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import static java.lang.Thread.sleep;

@Autonomous(name="BlueStoned", group ="11697")
//BLUE Left has "public class BlueLeft extends LinearOpMode
public class BlueStoned extends LinearOpMode {
    Hardware robot = new Hardware();
    /** PROGRAM CONFIGURATION VARIABLES */
    private static boolean closeNavWall = false;
    private static boolean closeSkystone = false;
    private static boolean dragFoundation = false;
    private static boolean closeParkWall = false;

    // Motors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Used by system functions
    private ElapsedTime moveTimer = new ElapsedTime();
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    private String PREVIOUS_MSG = "";
    private String stonePosition = "";

    /*
       Vuforia License Key can be found at:
       https://developer.vuforia.com/license-manager
     */

    private static final String VUFORIA_KEY = "ARYNigX/////AAABmeXRDofbC0DPuqRvnFl7Lm0xLd3BcZs7CL0R30y56aNmiqgHfjjzoNz1tmjW/mCUTlFCDlebwZwBvbsUYHVnFbvZDj4JAMqCWXjfeQGIPZLKg5ySTrg49Bx0UiaJybDMZhiJfdPAafR02xAXGXvnA8Uoa7/jomwlc2djsoG08Z2b5hKK83LT3nsWixPjixBedVs33VFKTF8w1oarxQwJEUf4GRvBeQVXqz7wgLIeMS1j8NKkcfDLQP1fPTPZINmkSI6vEutYGzPEGl827w37aUx6eszeaw/TEMVWeQFxIZdCwa8uEgPRph695Z1yJu4UoLy6mQXZMXMFTxmW+Z6SCKWIqmNPKLGnRtSNzQol2V9y";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //ADD VALUES FOR SKYSTONE DETECTION
    private int stoneX = -1;
    private int skystoneX = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }  else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                if (updatedRecognitions.size() == 2) {

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            stoneX = (int) recognition.getTop();
                        } else {
                            skystoneX = (int) recognition.getTop();
                        }
                    }

                    //NEW LOGIC FOR DETECTION

                    if(skystoneX == -1) {
                        stonePosition = "RIGHT";

                    } else {
                        if(skystoneX > stoneX)
                            stonePosition = "CENTER";

                        else
                            stonePosition = "LEFT";
                    }


                    telemetry.addData("The Skystone is on the: ", stonePosition);
                    telemetry.addData("skystoneX: ", skystoneX);
                    telemetry.addData("stoneX: ", stoneX);
                    telemetry.update();

                }

            }
        }

        waitForStart();


        while (opModeIsActive()) {

            //Move toward stones
            driveByEncoder(5,0,00);

            //Scan for skystones
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 2) {

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                stoneX = (int) recognition.getTop();
                            } else {
                                skystoneX = (int) recognition.getTop();
                            }
                        }

                        // Detection logic
                        if(skystoneX == -1) {
                            stonePosition = "RIGHT";

                        } else {
                            if(skystoneX > stoneX)
                                stonePosition = "CENTER";

                            else
                                stonePosition = "LEFT";
                        }

                        telemetry.addData("The Skystone is on the: ", stonePosition);
                        telemetry.addData("skystoneX: ", skystoneX);
                        telemetry.addData("stoneX: ", stoneX);
                        telemetry.update();

                    }
                }
            }

            sleep(5000);

            switch(stonePosition) {
                case "LEFT":
                    //Move to skystone

                    //Grab Skystone

                    //Move away from skystone

                    break;


                case "CENTER":
                    //Move to skystone

                    //Grab skystone

                    //Move away from skystone

                    break;

                case "RIGHT":
                    //Move to skystone

                    //Grab skystone

                    //Move away from skystone

                    break;

                default:

                    //*INSERT CENTER CODE HERE*
                    break;

            }

                if(closeSkystone) {
                    //Adjust as necesssary

                } else {
                    //Adjust as necessary
                }


                if(closeNavWall) {
                    //Adjust as necessary


                } else {
                    //Adjust as necessary
                }

                //Drive under Skybridge


                if(dragFoundation) {
                    //Drag Foundation

                } else {
                    //Adjust

                }

                //Code for placing Skystone

                if(closeParkWall) {

                } else {
                    
                }







            sleep(5000);
            break;


        }

    }



    /*
     **************************************************
     **************USER DEFINED FUNCTIONS:*************
     **************************************************
     */

    private void driveByEncoder (double speed, double leftInches, double rightInches) throws InterruptedException {

        /**********************************************************
         driveByEncoder(0.3, 10.0, 10.0);            // Forward
         driveByEncoder(0.3, -10.0, -10.0);          // Backward
         driveByEncoder(0.3, 0, 10.0);               // Shift Right
         driveByEncoder(0.3, 10.0, 0);               // Shift Left
         driveByEncoder(0.3, 22.0, -22.0);           // Turn Right
         driveByEncoder(0.3, -22.0, 22.0);           // Turn Left
         ***********************************************************/

        String robotAction = "";
        int newLeftTarget;
        int newRightTarget;

        if (leftInches < 0 && rightInches < 0) {
            robotAction = "BACKWARD";
        } else if (leftInches > 0 && rightInches > 0) {
            robotAction = "FORWARD";
        } else if (leftInches > 0 && rightInches == 0) {
            robotAction = "SHIFT_LEFT";
        } else if (leftInches == 0 && rightInches > 0) {
            robotAction = "SHIFT_RIGHT";
        } else if (leftInches < 0 && rightInches > 0) {
            robotAction = "TURN_LEFT";
        } else if (leftInches > 0 && rightInches < 0) {
            robotAction = "TURN_RIGHT";
        } else {
            return;
        }

        // Remember current motors direction, will reset in the end
        DcMotor.Direction dirFL = robot.frontLeftMotor.getDirection();
        DcMotor.Direction dirFR = robot.frontRightMotor.getDirection();
        DcMotor.Direction dirRL = robot.rearLeftMotor.getDirection();
        DcMotor.Direction dirRR = robot.rearRightMotor.getDirection();
        DcMotor.RunMode runModeFL = robot.frontLeftMotor.getMode();
        DcMotor.RunMode runModeFR = robot.frontRightMotor.getMode();
        DcMotor.RunMode runModeRL = robot.rearLeftMotor.getMode();
        DcMotor.RunMode runModeRR = robot.rearRightMotor.getMode();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // power is removed from the motor, set the current encoder position to zero
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All mortors will move forward
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = (int)(rightInches * COUNTS_PER_INCH);
        //logMessage("curFL,curFR",  robot.frontLeftMotor.getCurrentPosition() +", "+ robot.frontRightMotor.getCurrentPosition());

        switch(robotAction) {

            case "FORWARD":
                //logMessage("Moving Robot", "FORWARD");
                break;

            case "BACKWARD": // mortor direction aame as FORWAED, because encoder will be "-"
                //logMessage("Moving Robot", "BACKWARD");
                break;

            case "SHIFT_LEFT":
                //logMessage("Moving Robot", "SHIFT_LEFT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //+
                robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //+
                robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //+
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //+
                //robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                newRightTarget = newLeftTarget;
                break;

            case "SHIFT_RIGHT":
                //logMessage("Moving Robot", "SHIFT_RIGHT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //-
                robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //-
                robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //-
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //-
                //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                newLeftTarget = newRightTarget;
                break;

            case "TURN_LEFT":
                //logMessage("Moving Robot", "TURN_LEFT");
                break;

            case "TURN_RIGHT":
                //logMessage("Moving Robot", "TURN_RIGHT");
                break;

        }

        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
        robot.frontRightMotor.setTargetPosition(newRightTarget);
        robot.rearLeftMotor.setTargetPosition(newLeftTarget);
        robot.rearRightMotor.setTargetPosition(newRightTarget);
        //logMessage("newLeftTarget,newRightTarget",  newLeftTarget +", "+ newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the motor speed and start motion.
        robot.frontLeftMotor.setPower(Math.abs(speed));
        robot.frontRightMotor.setPower(Math.abs(speed));
        robot.rearLeftMotor.setPower(Math.abs(speed));
        robot.rearRightMotor.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {

            /*
            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
            logMessage("Path2",
                    robot.frontLeftMotor.getCurrentPosition() + ", " +
                    robot.frontRightMotor.getCurrentPosition() + ", " +
                            robot.rearLeftMotor.getCurrentPosition() + ", " +
                            robot.rearRightMotor.getCurrentPosition());
            */
        }


        //logMessage("FL,FR,RL,RR",
        //        robot.frontLeftMotor.getCurrentPosition() + ", " +
        //                robot.frontRightMotor.getCurrentPosition() + ", " +
        //                robot.rearLeftMotor.getCurrentPosition() + ", " +
        //                robot.rearRightMotor.getCurrentPosition());

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset back motors direction
        robot.frontLeftMotor.setDirection(dirFL);
        robot.frontRightMotor.setDirection(dirFR);
        robot.rearLeftMotor.setDirection(dirRL);
        robot.rearRightMotor.setDirection(dirRR);
        robot.frontLeftMotor.setMode(runModeFL);
        robot.frontRightMotor.setMode(runModeFR);
        robot.rearLeftMotor.setMode(runModeRL);
        robot.rearRightMotor.setMode(runModeRR);

    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    //Movement path for rightmost two blocks
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

//ORIGINAL CONFIDENCE 0.8
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}



