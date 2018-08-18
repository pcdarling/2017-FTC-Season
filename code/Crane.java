package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Shelby Goss on 10/14/2017.
 */

@Disabled
@TeleOp(name="Crane", group="TeamBot")
public class Crane extends LinearOpMode {

    public CraneHardware robot = new CraneHardware();

    // Preference Constants
    double STANDARD_SPEED = 0.3;
    double STANDARD_SHOULDER_SPEED = 0.25;
    double LOWER_SHOULDER_SPEED = 0.05;
    double INC_SERVO = 0.1;
    int INC_SHOULDER = 1;
    int INC_EXTENDER = 1;
    int OPEN_POS_NABBER = 175;
    int CLOSE_POS_NABBER = 140;

    // Measured Constants
    int COUNTS_TO_HORIZONTAL = 900;
    int COUNTS_TO_FIRST_LEVEL = 100;
    int COUNTS_TO_FULLY_EXTENDED = 2200;

    // Hardware Constants
    double RETRACTED_LENGTH = 16; // inches
    double IDOL_HEIGHT = 10; // inches
    double GRABBER_REDUCTION = 1.5; // inches
    double WALL_HEIGHT = 12; // inches
    int DRIVE_GEAR_REDUCTION = 30;
    int PULSES_PER_REV = 28;

    // Calculated Constants
    int COUNT_PER_REV = PULSES_PER_REV * DRIVE_GEAR_REDUCTION; // specifically for 40:1
    int COUNTS_PER_DEGREE = COUNTS_TO_HORIZONTAL/90;
    double HOLD_HEIGHT = (IDOL_HEIGHT-GRABBER_REDUCTION)+WALL_HEIGHT;
    double REQUIRED_ANGLE_FROM_HORIZONTAL = Math.asin(HOLD_HEIGHT/RETRACTED_LENGTH);
    int COUNTS_TO_OVER_WALL = (int)Math.ceil(REQUIRED_ANGLE_FROM_HORIZONTAL*COUNTS_PER_DEGREE);
    int COUNTS_TO_NEXT_LEVEL = (int)Math.ceil((COUNTS_TO_FULLY_EXTENDED-COUNTS_TO_FIRST_LEVEL)/3);

    // State of machine
    int shoulderMode = 0;
    int extensionMode = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        setupRobot();

        waitForStart();

        boolean left = false;
        boolean right = false;
        boolean up = false;
        boolean down = false;

        while (opModeIsActive()){
            if(gamepad1.y){
                // Raise Shoulder
                rotateShoulder(0,STANDARD_SHOULDER_SPEED);
            }
            if (gamepad1.a) {
                // Lower Shoulder
                rotateShoulder(1,LOWER_SHOULDER_SPEED);
            }
            if (gamepad1.b){
                // Completely Close Nabber
                grabRelease(2);
            }
            if (gamepad1.x){
                // Completely Open Nabber
                grabRelease(3);
            }

            // Wait for instructions
            idle();
        }
        shutdownRobot();
    }

    public void rotateShoulder (int decision, double speed){
        int newPos;
        if (decision == 0 || decision == 1 || decision == 2 && shoulderMode < 0) {
            // Moving from manual to mode select (simple---Go Home first)
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            shoulderMode = 0;
            newPos = 0;
        }
        else if (decision == 0 && shoulderMode < 2 && shoulderMode >= 0) {
            // Rotate towards over-the-wall position
            robot.Shoulder.setDirection(DcMotor.Direction.FORWARD);
            if (shoulderMode == 0) {
                // Encoder should be around 0
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_HORIZONTAL;
            } else {
                // Encoder should be around COUNTS_TO_HORIZONTAL
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_OVER_WALL;
            }
            shoulderMode++;
        } else if (decision == 1 && shoulderMode > 0) {
            // Rotate towards resting position
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            if (shoulderMode == 2) {
                // Encoder should be around COUNTs_TO_HORIZONTAL+COUNTS_TO_OVER_WALL
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_OVER_WALL;
            } else {
                // Encoder should be around COUNTS_TO_HORIZONTAL
                newPos = 0;
            }
            shoulderMode--;
        } else if (decision == 2) {
            // Set the arm back to its resting position
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            newPos = 0;
            shoulderMode = 0;
        } else {
            // Manually rotating shoulder
            shoulderMode = -1;
            if(speed < 0){
                robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                robot.Shoulder.setDirection(DcMotor.Direction.FORWARD);
            }
            newPos = robot.Shoulder.getCurrentPosition() + INC_SHOULDER;
        }

        robot.Shoulder.setTargetPosition(newPos);

        robot.Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        robot.Shoulder.setPower(speed);

        while(opModeIsActive() && robot.Shoulder.isBusy()){
            //busy waiting
            telemetry.addData("Shoulder Status:", "Rotating");
        }

        robot.Shoulder.setPower(0);
        robot.Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendRetract(int decision, double speed){
        int newPos;
        if (decision == 0 || decision == 1 || decision == 2 && shoulderMode < 0) {
            // Moving from manual to mode select (simple---Go Home first)
            grabRelease(3);
            grabRelease(2);
            robot.Extender.setDirection(DcMotor.Direction.REVERSE);
            extensionMode = 0;
            newPos = 0;
        }
        else if (decision == 0 && extensionMode < 3 && extensionMode >= 0) {
            // Extend Arm
            robot.Extender.setDirection(DcMotor.Direction.FORWARD);
            if (extensionMode == 0) {
                // Get to first level
                newPos = robot.Extender.getCurrentPosition() + COUNTS_TO_FIRST_LEVEL;
            } else {
                // Rest of the levels are in equal measurements
                newPos = robot.Extender.getCurrentPosition() + COUNTS_TO_NEXT_LEVEL;
            }
            extensionMode++;
        } else if (decision == 1 && extensionMode > 0) {
            // Retract Arm
            robot.Extender.setDirection(DcMotor.Direction.REVERSE);
            if (extensionMode == 1) {
                // Send arm home
                newPos = 0;
            } else {
                // Rest of the levels are in equal measurements
                newPos = robot.Extender.getCurrentPosition() + COUNTS_TO_NEXT_LEVEL;
            }
            extensionMode--;
        } else if (decision == 2) {
            // Fully retract Arm
            robot.Extender.setDirection(DcMotor.Direction.REVERSE);
            extensionMode = 0;
            newPos = 0;
        } else {
            // Manually Control Arm
            extensionMode = -1;
            if (speed < 0) {
                robot.Extender.setDirection(DcMotor.Direction.REVERSE);
            } else {
                robot.Extender.setDirection(DcMotor.Direction.FORWARD);
            }
            newPos = robot.Extender.getCurrentPosition() + INC_EXTENDER;
        }

        robot.Extender.setTargetPosition(newPos);

        robot.Extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.Extender.setPower(speed);

        while (opModeIsActive() && robot.Extender.isBusy()){
            telemetry.addData("MotorPosition", "%d", robot.Extender.getCurrentPosition());
            telemetry.update();
        }


        robot.Extender.setPower(0);
        robot.Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void grabRelease (int decision){
        if (decision == 0){
            //does incremental close
            double cur = robot.Nabber.getPosition();
            robot.Nabber.setPosition(cur + INC_SERVO);
        }
        else if (decision == 1){
            //does incremental open
            double cur = robot.Nabber.getPosition();
            robot.Nabber.setPosition(cur - INC_SERVO);
        }
        else if (decision == 2){
            //does closes
            robot.Nabber.setPosition(CLOSE_POS_NABBER);
        }
        else {
            //does opens
            robot.Nabber.setPosition(OPEN_POS_NABBER);
        }
    }

    public void setupRobot() {
        robot.Nabber.setPosition(OPEN_POS_NABBER);
    }

    public void shutdownRobot() {
        // Close the Nabber
        robot.Nabber.setPosition(CLOSE_POS_NABBER);

        // Fully retract Extender
        extendRetract(2,STANDARD_SPEED);

        // Fully lower Shoulder
        rotateShoulder(2,LOWER_SHOULDER_SPEED);
    }

}
