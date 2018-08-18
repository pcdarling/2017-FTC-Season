package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Shelby Goss on 10/14/2017.
 */

@Disabled
@TeleOp(name="Drivingprogram", group= "TeamBot")
public class DrivingProgram extends LinearOpMode {

    public DrivetrainHardware robot = new DrivetrainHardware();

    // Preference Constants
    double STANDARD_SPEED = 0.2;
    double MINIMUM_SPEED = 0.1;

    // Hardware Constants
    int DRIVE_GEAR_REDUCTION = 10;
    int PULSES_PER_REV = 28;

    double WHEEL_DIAMETER = 4.875; // inches
    double ROBOT_LENGTH = 17.75;
    double ROBOT_WIDTH = 13.5;
    double ROBOT_DIAMETER = Math.sqrt(Math.pow(ROBOT_LENGTH,2) + Math.pow(ROBOT_WIDTH,2));
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
    double COUNTS_PER_INCH = PULSES_PER_REV * DRIVE_GEAR_REDUCTION/(Math.PI*WHEEL_DIAMETER);
    int COUNTS_PER_REV = PULSES_PER_REV*DRIVE_GEAR_REDUCTION;
    double NUM_ROTATIONS = ROBOT_CIRCUMFERENCE/(2 * WHEEL_CIRCUMFERENCE);
    double COUNTS_TO_180 = NUM_ROTATIONS*COUNTS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        waitForStart();

        double Y;
        double R;
        while (opModeIsActive()){
            // Save intended commands
            Y = gamepad1.left_stick_y;
            R = gamepad1.right_stick_x;

            // Check if wanting to do a preset movement
            if (gamepad1.b) {
                turn180(0.2);
            } else if (gamepad1.x){
                turn180(-0.2);
            }

            // Check if wanting to drive
            if (Math.abs(Y) < Math.abs(R)) {
                // If intent to rotate is more than intent to drive
                if (Math.abs(R) < MINIMUM_SPEED) {
                    // If not fast enough
                    setPowerToAllMotors(0);
                }
                else {
                  rotate(R);
                }
            } else {
                if (Math.abs(Y) < MINIMUM_SPEED) {
                    setPowerToAllMotors(0);
                } else {
                    setPowerToAllMotors(Y);
                }
            }

            // Wait for instructions
            idle();
        }

    }

    public void turn180 (double speed){
        int newPosFL;
        int newPosFR;
        int newPosRL;
        int newPosRR;
        robot.FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newPosFL = robot.FLmotor.getCurrentPosition() + (int)COUNTS_TO_180;
        newPosFR = robot.FRmotor.getCurrentPosition() + (int)COUNTS_TO_180;
        newPosRL = robot.RLmotor.getCurrentPosition() + (int)COUNTS_TO_180;
        newPosRR = robot.RRmotor.getCurrentPosition() + (int)COUNTS_TO_180;

        robot.FLmotor.setTargetPosition(newPosFL);
        robot.FRmotor.setTargetPosition(newPosFR);
        robot.RLmotor.setTargetPosition(newPosRL);
        robot.RRmotor.setTargetPosition(newPosRR);

        robot.FLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(speed < 0){
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);
        }

        speed = Math.abs(speed);
        robot.FLmotor.setPower(speed);
        robot.FRmotor.setPower(speed);
        robot.RLmotor.setPower(speed);
        robot.RRmotor.setPower(speed);

        while(opModeIsActive() && robot.FLmotor.isBusy() && robot.FRmotor.isBusy() &&
                robot.RLmotor.isBusy() && robot.RRmotor.isBusy()){
            //busy waiting
            telemetry.addData("Wheel Status:", "Turning 180");
        }

        robot.FLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void rotate(double power){
        if(power<0){
            power = Math.abs(power);
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);

            robot.FLmotor.setPower(power);
            robot.RLmotor.setPower(power);
            robot.FRmotor.setPower(power);
            robot.RRmotor.setPower(power);
        } else {
            power = Math.abs(power);
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);

            robot.FLmotor.setPower(power);
            robot.RLmotor.setPower(power);
            robot.FRmotor.setPower(power);
            robot.RRmotor.setPower(power);
        }
    }


    public void setPowerToAllMotors(double power){
        // also defines direction
        if(power<0){
            power = Math.abs(power);
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);

            robot.FLmotor.setPower(power);
            robot.RLmotor.setPower(power);
            robot.FRmotor.setPower(power);
            robot.RRmotor.setPower(power);
        } else {
            power = Math.abs(power);
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);

            robot.FLmotor.setPower(power);
            robot.RLmotor.setPower(power);
            robot.FRmotor.setPower(power);
            robot.RRmotor.setPower(power);
        }
    }

}
