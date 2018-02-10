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



package org.firstinspires.ftc.teamcode.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.provider.Settings;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous(name="BlueSouth", group="Linear Opmode")
public class SouthWestJewelTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private Servo jewel = null;
    private DcMotor tilt = null;
    private DcMotor up = null;
    private DcMotor pinch = null;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    ColorSensor colorSensor;
    ColorSensor colorSensorB;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        jewel = hardwareMap.get(Servo.class, "jewel");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensorB = hardwareMap.get(ColorSensor.class, "colorB");
        tilt  = hardwareMap.get(DcMotor.class, "tilt");
        up  = hardwareMap.get(DcMotor.class, "up");
        pinch  = hardwareMap.get(DcMotor.class, "pinch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        tilt.setDirection(DcMotor.Direction.FORWARD);
        up.setDirection(DcMotor.Direction.REVERSE);
        pinch.setDirection(DcMotor.Direction.FORWARD);

        /*
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        boolean bLedOn = true;


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        boolean active = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && active) {
            //Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            colorSensor.enableLed(bLedOn);

            //Arm Code
            ColorSensorJewelArm2("Blue");
            sleep(250);
            VuMarkImageDetector();
            sleep(250);
            //Driving Code
            MoveForward(28, 0.25);
            sleep(250);
            TurnLeft(92, 0.3);
            sleep(250);
            MoveForward(16, 0.25);
            sleep(250);
            pinch.setPower(1);
            sleep(750);
            pinch.setPower(0);
            sleep(250);
            MoveBackward(5.55, 0.3);
            sleep(250);
            pinch.setPower(1);
            sleep(750);
            pinch.setPower(0);
            sleep(250);
            MoveBackward(5.55, 0.3);
            sleep(250);

            active = false;
        }

        leftPower    = 0 ;
        rightPower   = 0 ;
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);


    }
    public void MoveForward(double inches, double speed) {
        double Power = speed;
        long time = (long)(inches * 65);//(semi precise)
        frontLeftDrive.setPower(Power);
        frontRightDrive.setPower(Power);
        backLeftDrive.setPower(Power);
        backRightDrive.setPower(Power);
        telemetry.addData("Move Forward", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    public void MoveBackward(double inches, double speed) {
        double Power = speed;
        long time = (long)(inches * 65);//(semi precise)
        frontLeftDrive.setPower(Power * -1);
        frontRightDrive.setPower(Power * -1);
        backLeftDrive.setPower(Power * -1);
        backRightDrive.setPower(Power * -1);
        telemetry.addData("Move Backward", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    public void TurnLeft(double degrees, double speed) {
        double Power = speed;
        long time = (long)(degrees * 14.9);//(precise)
        frontLeftDrive.setPower(Power * -1);
        frontRightDrive.setPower(Power);
        backLeftDrive.setPower(Power * -1);
        backRightDrive.setPower(Power);
        telemetry.addData("Left Turn", "Degrees (" + degrees + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void TurnRight(double degrees, double speed) {
        double Power = speed;
        long time = (long)(degrees * 14.9);//(precise)
        frontLeftDrive.setPower(Power);
        frontRightDrive.setPower(Power * -1);
        backLeftDrive.setPower(Power);
        backRightDrive.setPower(Power * -1);
        telemetry.addData("Right Turn", "Degrees (" + degrees + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void MoveLeft(double inches, double speed) {
        double LeftPower = speed;
        double RightPower = speed;
        long time = (long)(inches * 85);//NOT THE REAL VALUE (imprecise)
        frontLeftDrive.setPower(LeftPower * -1);
        frontRightDrive.setPower(RightPower);
        backLeftDrive.setPower(LeftPower);
        backRightDrive.setPower(RightPower * -1);
        telemetry.addData("Move Left", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void MoveRight(double inches, double speed) {
        double LeftPower = speed;
        double RightPower = speed;
        long time = (long) (inches * 85);//NOT THE REAL VALUE (imprecise)
        frontLeftDrive.setPower(LeftPower);
        frontRightDrive.setPower(RightPower * -1);
        backLeftDrive.setPower(LeftPower * -1);
        backRightDrive.setPower(RightPower);
        telemetry.addData("Move Right", "Inches (" + inches + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void ColorSensorJewelArm() {
        boolean detectRed = false;
        boolean detectBlue = false;
        boolean detectNothing = false;
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        jewel.setPosition(0.582);
        sleep(400);

        // get a reference to our ColorSensor object.

        // Set the LED in the beginning
        colorSensor.enableLed(true);
        sleep(500);
        //We are on blue team in this one
        if (colorSensor.blue() < colorSensor.red() && colorSensor.red() > 1) {
            detectRed = true;
        } else if (colorSensor.red() < colorSensor.blue() && colorSensor.blue() > 1) {
            detectBlue = true;
        } else detectNothing = true;
        sleep(1);
        if (detectRed){
            TurnLeft(20, 0.25);
            sleep(500);
            jewel.setPosition(0);
            TurnRight(20, 0.25);
        }
        if (detectBlue){
            TurnRight(20, 0.25);
            sleep(500);
            jewel.setPosition(0);
            TurnLeft(20, 0.25);
        }
        else sleep(1);
        sleep(500);
        jewel.setPosition(0);
        sleep(250);
        if (detectNothing){
            MoveForward(2, 0.25);
            jewel.setPosition(0.582);
            sleep(250);
            if (colorSensor.blue() < colorSensor.red() && colorSensor.red() > 1) {
                detectRed = true;
            } else if (colorSensor.red() < colorSensor.blue() && colorSensor.blue() > 1) {
                detectBlue = true;
            } else detectNothing = true;
            sleep(1);
            if (detectRed){
                TurnLeft(20, 0.25);
                sleep(500);
                jewel.setPosition(0);
                TurnRight(20, 0.25);
            }
            if (detectBlue){
                TurnRight(20, 0.25);
                sleep(500);
                jewel.setPosition(0);
                TurnLeft(20, 0.25);
            }
            jewel.setPosition(0.582);
            sleep(250);
        }
        jewel.setPosition(0);
        detectBlue = false;
        detectRed = false;
        sleep(500);

    }
    public void ColorSensorJewelArm2(String sideColor) {
        boolean color2 = true;
        String detectColor = "StartingValue";
        String detectColor2 = "StartingValue";
        int loopColorCount = 0;
        jewel.setPosition(0.85);
        sleep(400);
        colorSensor.enableLed(true);
        sleep(500);
        while (loopColorCount < 5) {
            if (color2){
                if (colorSensorB.blue() < colorSensorB.red() && colorSensorB.red() > 1){
                    detectColor2 = "Red";
                }
                else if (colorSensorB.red() < colorSensorB.blue() && colorSensorB.blue() > 1) {
                    detectColor2 = "Blue";
                }
                else detectColor2 = "Nothing";
            }
            else sleep(1);
            if (colorSensor.blue() < colorSensor.red() && colorSensor.red() > 1) {
                detectColor = "Red";
            } else if (colorSensor.red() < colorSensor.blue() && colorSensor.blue() > 1) {
                detectColor = "Blue";
            } else detectColor = "Nothing";

            //End Detection
            sleep(10);
            if (detectColor.equals("Red") || detectColor2.equals("Blue")){
                if(sideColor.equals("Red")){
                    MoveBackward(3.25, 0.2);
                    sleep(250);
                    jewel.setPosition(0);
                    sleep(250);
                    MoveForward(3.25, 0.25);
                }
                else if(sideColor.equals("Blue")){
                    MoveForward(3.25, 0.25);
                    sleep(250);
                    jewel.setPosition(0);
                    sleep(250);
                    MoveBackward(3.25, 0.25);
                }
                else jewel.setPosition(0);
                loopColorCount = 6;
            }
            if (detectColor.equals("Blue") || detectColor2.equals("Red")){
                if(sideColor.equals("Blue")){
                    MoveBackward(3.25, 0.2);
                    sleep(250);
                    jewel.setPosition(0);
                    sleep(250);
                    MoveForward(3.25, 0.25);
                }
                else if(sideColor.equals("Red")){
                    MoveForward(3.25, 0.25);
                    sleep(250);
                    jewel.setPosition(0);
                    sleep(250);
                    MoveBackward(3.25, 0.25);
                }
                else jewel.setPosition(0);
                loopColorCount = 6;
            }
            else if (detectColor.equals("Nothing") && detectColor2.equals("Nothing")) {
                MoveForward(1.1,0.15);
                sleep(250);
            }
            else sleep(1);
            sleep(250);
            loopColorCount += 1;
        }
        jewel.setPosition(0.05);
        detectColor = "EndingValue";
        sleep(250);


    }
    public String VuMarkImageDetector() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdCD63D/////AAAAGThVVvAi9UMXrbAu6IouvThcGKupmnakuvoWHZtIOH78mz1zQ+JAsVe7NqsffG4WpT1W2DvJQ8VsniObDD0N2W6y7WeavS8kseppMEdzy22UdVDXzvfPfoK/l62C3x0esCe7xeM8IOwZW8GtJX6cOalAR5HgYuS3VuN8eE/sPD9RmYwwRkhkGOntMOlWxc8yCIwTnn3nYBGEsOFEpz2+R+YboSIX2jWL1xs6Z7YqnA2rAAX489xbIoCsTWZEzQlPfbXk7frpTZpT7Nq3kh1PeGcRg536UTWGJ69fSRr8PIHJdycexY7uPhmfhEZBy3/pFOZ5lNhnqBuekll8PAhjftnvXeyRiWHugSEjVNUzdWhY";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;//We indicate which camera on the RC that we wish to use.
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        boolean imgScan = true;
        int loopCount = 0;
        String picto = "none";

        while (imgScan && loopCount >= 100) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
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
                if (vuMark == RelicRecoveryVuMark.LEFT){
                    picto = "LEFT";
                    telemetry.addData("VuMark", "Left");
                    imgScan = false;
                    return picto;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER){
                    picto = "CENTER";
                    telemetry.addData("VuMark", "Center");
                    imgScan = false;
                    return picto;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    picto = "RIGHT";
                    telemetry.addData("VuMark", "Right");
                    imgScan = false;
                    return picto;
                }
                else sleep(100);

            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            loopCount = loopCount + 1;
        }
        return picto;
    }
    public void ImgDistDec(String pictod){
        if (pictod.equals("LEFT")){
            MoveLeft(6, 0.25);
            MoveForward(2, 0.25);
        }
        if (pictod.equals("CENTER")){
            MoveForward(2, 0.25);
        }
        if (pictod.equals("RIGHT")){
            MoveRight(6, 0.25);
        }
        else{
            telemetry.addData("Image Not Found", pictod);
            telemetry.update();
        }
    }
    String format (OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

