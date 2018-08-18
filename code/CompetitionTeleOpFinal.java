package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Prestolie on 1/29/2018.
 */

@TeleOp(name="competitionfinal",group="TeamBot")
public class CompetitionTeleOpFinal extends LinearOpMode {

    public CompetitionHardware2 robot = new CompetitionHardware2();

    // Preference Constants
    double MINIMUM_SPEED = 0.1;
    double STANDARD_SPEED = 0.2;
    double STANDARD_SHOULDER_SPEED = 0.18;
    double STANDARD_ELEVATOR_SPEED = 1;
    double INC_SERVO = 1 / 255;
    double STANDARD_CLAW_SPEED = 0.5;
    double X_HOME = 0.5;
    double Y_HOME = 0.7;
    double Y_DOWN = 0.1;
    double X_LEFT = 0.2;
    double X_RIGHT = 0.8;

    // Measured Constants
    int ELEVATOR_MAX = 6100;

    // State of machine
    boolean craneControls = false;
    int drivingDirection = 1;

    // Threads
    ClawThread ct = new ClawThread(robot, STANDARD_CLAW_SPEED);
    DriveThread dt = new DriveThread(robot, 0, STANDARD_SPEED, false);
    ElevatorThread et = new ElevatorThread(robot, STANDARD_ELEVATOR_SPEED, true);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        setupRobot();

        waitForStart();

        while (opModeIsActive()) {
            // Check Driver (gamepad1) controls
            checkDriverControls();

            // Check Operator (gamepad2) controls
            checkOperatorControls();

            telemetry.addData("Drivethread Status: ", dt.isAlive());
            telemetry.addData("Clawthread Status: ", ct.isAlive());
            if (drivingDirection < 0) {
                telemetry.addData("Front of Bot: ", "Helping Hand");
            } else {
                telemetry.addData("Front of Bot: ", "Elevator");
            }
            if (craneControls) {
                telemetry.addData("Operator Mode: ", "Crane Controls");
            } else {
                telemetry.addData("Operator Mode: ", "Elevator Controls");
            }
            telemetry.update();

            // Wait for instructions
            idle();
        }
        shutdownRobot();
    }

    private void checkDriverControls() {
        double rightY = gamepad1.right_stick_y;
        double leftY = gamepad1.left_stick_y;

        if(Math.abs(rightY) > 0.06) {
            robot.FRmotor.setPower(rightY);
            robot.RRmotor.setPower(rightY);
        }
        else {
            robot.FRmotor.setPower(0);
            robot.RRmotor.setPower(0);
        }
        if(Math.abs(leftY) > 0.06){
            robot.FLmotor.setPower(leftY);
            robot.RLmotor.setPower(leftY);
        }
        else {
            robot.FLmotor.setPower(0);
            robot.RLmotor.setPower(0);
        }
        return;
    }

    private void checkOperatorControls() {

        double R_Y = gamepad2.right_stick_y;
        double L_Y = gamepad2.left_stick_y;
        boolean botButtonPressed = robot.tBot.isPressed();
        boolean backPressed = gamepad2.back;
        boolean leftBumperPressed = gamepad2.left_bumper;
        boolean rightBumperPressed = gamepad2.right_bumper;
        boolean upPressed = gamepad2.dpad_up;
        boolean downPressed = gamepad2.dpad_down;
        boolean leftPressed = gamepad2.dpad_left;
        boolean rightPressed = gamepad2.dpad_right;
        double leftTrigger = gamepad2.left_trigger;
        double rightTrigger = gamepad2.right_trigger;


        if (backPressed) {
            craneControls = !craneControls;
        }
        if (craneControls) {
            // Crane Commands
            if (Math.abs(R_Y) < MINIMUM_SPEED) {
                robot.Shoulder.setPower(0);
            } else {
                robot.Shoulder.setPower(R_Y*STANDARD_SHOULDER_SPEED) ;
            }
            if (downPressed) {
                robot.gemY.setPosition(Y_DOWN);
            }
            if (upPressed) {
                robot.gemY.setPosition(Y_HOME);
            }
            if (leftPressed) {
                robot.gemX.setPosition(X_LEFT);
            }
            if (rightPressed) {
                robot.gemX.setPosition(X_RIGHT);
            }
            if (rightBumperPressed || leftBumperPressed){
                robot.gemX.setPosition(X_HOME);
            }
            telemetry.addData("Mode","Crane/Gem");
        } else {
            // Elevator Commands
            if (!et.isAlive()) {
                if (Math.abs(L_Y) < MINIMUM_SPEED) {
                    // If the stick is telling the elevator to go to slow, don't move
                    robot.Elevator.setPower(0);
                } else if ((botButtonPressed && L_Y > 0) || (robot.Elevator.getCurrentPosition() < -ELEVATOR_MAX && L_Y < 0)) {
                    // If the elevator is already down all the way and the user is trying to go down, stay
                    robot.Elevator.setPower(0);
                    if (robot.Elevator.getCurrentPosition() != 0 && botButtonPressed) {
                        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                } else {
                    // The robot is ok to move
                    robot.Elevator.setPower(L_Y);
                    robot.elevatorMode = -1;
                }
                if (gamepad2.y){
                    // Up
                    et = new ElevatorThread(robot, -STANDARD_ELEVATOR_SPEED, false);
                    et.start();
                }
                if (gamepad2.a){
                    // Down
                    et = new ElevatorThread(robot, STANDARD_ELEVATOR_SPEED, false);
                    et.start();
                }
            }
            // Claw manual
            if (!ct.isAlive()) {
                if (leftBumperPressed){
                    robot.Cleft.setPower(-STANDARD_CLAW_SPEED);
                }
                else if (leftTrigger > 0.2){
                    robot.Cleft.setPower(STANDARD_CLAW_SPEED);
                }
                else if (rightBumperPressed){
                    robot.Cright.setPower(STANDARD_CLAW_SPEED);
                }
                else if (rightTrigger > 0.2){
                    robot.Cright.setPower(-STANDARD_CLAW_SPEED);
                }
                else if (Math.abs(R_Y) < MINIMUM_SPEED) {
                    // Too slow claw
                    robot.Cleft.setPower(0);
                    robot.Cright.setPower(0);
                } else {
                    robot.Cright.setPower(R_Y * STANDARD_CLAW_SPEED);
                    robot.Cleft.setPower(-R_Y * STANDARD_CLAW_SPEED);
                }
                //individual claw movement
                if (gamepad2.b) {
                    // Close claw
                    ct = new ClawThread(robot,-STANDARD_CLAW_SPEED);
                    ct.start();
                }
                if (gamepad2.x) {
                    // Open claw
                    ct = new ClawThread(robot,STANDARD_CLAW_SPEED);
                    ct.start();
                }
            }
            telemetry.addData("Mode","Elevator");
        }
        telemetry.update();
    }

    // Below this point are "helper functions" meaning that they don't do anything special for
    // controlling the robot. They just do some functions that we use a lot.

    private void setModeToDrivetrain(DcMotor.RunMode mode) {
        robot.FLmotor.setMode(mode);
        robot.FRmotor.setMode(mode);
        robot.RLmotor.setMode(mode);
        robot.RRmotor.setMode(mode);
    }

    private void setPowerToDrivetrain(double leftPower, double rightPower) {
        robot.FLmotor.setPower(leftPower);
        robot.RLmotor.setPower(leftPower);
        robot.FRmotor.setPower(rightPower);
        robot.RRmotor.setPower(rightPower);
    }

    private void setupRobot() {
        ct = new ClawThread(robot,STANDARD_CLAW_SPEED);
        ct.start();

        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setModeToDrivetrain(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.gemX.setPosition(X_HOME);
        robot.gemY.setPosition(Y_HOME);
    }

    private void shutdownRobot() {
        // Open the claw
        ct = new ClawThread(robot,STANDARD_CLAW_SPEED);
        ct.start();
    }

    private static class ClawThread extends Thread {
        // Always True
        int OPEN_CLEFT = 0;
        int OPEN_CRIGHT = 0;
        int CLOSE_CLEFT = -268;
        int CLOSE_CRIGHT = 268;

        // Created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        double speed;

        public ClawThread(CompetitionHardware2 robot, double speed) {
            this.robot = robot;
            this.speed = speed;
        }
        public void run() {
            try {
                openClose(this.speed);
            } catch (Exception e) {
                // Something bad happened
                //System.out.println(e.toString());
            }
        }
        private void openClose(double speed) {
            double leftSpeed = speed;
            double rightSpeed = -speed;
            if (speed < 0){
                //does incremental close
                robot.Cleft.setTargetPosition(CLOSE_CLEFT);
                robot.Cright.setTargetPosition(CLOSE_CRIGHT);
            }
            else {
                //does incremental open
                robot.Cleft.setTargetPosition(OPEN_CLEFT);
                robot.Cright.setTargetPosition(OPEN_CRIGHT);
            }

            robot.Cleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Cright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.Cleft.setPower(leftSpeed);
            robot.Cright.setPower(rightSpeed);

            runtime = new ElapsedTime();
            while(!isInterrupted() && robot.Cleft.isBusy() && robot.Cright.isBusy()) {
                // busy waiting
                if (robot.tLeft.isPressed() && speed > 0) {
                    break;
                }
                if (robot.tRight.isPressed() && speed > 0) {
                    break;
                }
                if (runtime.seconds() > 1) {
                    break;
                }
            }

            if(robot.tLeft.isPressed()) {
                robot.Cleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.tRight.isPressed()){
                robot.Cright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            robot.Cleft.setPower(0);
            robot.Cright.setPower(0);

            robot.Cleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Cright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private static class DriveThread extends Thread {
        // Hardware Constants
        double WHEEL_DIAMETER = 5; // inches
        double ROBOT_LENGTH = 18;
        double ROBOT_WIDTH = 13; // Wheels are roughly 2.5 inches in
        int PULSES_PER_REV = 24; // After the internal gearbox
        int ENCODER_ERROR = 0;
        int DRIVE_GEAR_REDUCTION = (20*2+ENCODER_ERROR); // Using 20's with a 2:1 on the outside

        // Calculated Constants
        double ROBOT_DIAMETER = Math.sqrt(Math.pow(ROBOT_LENGTH,2) + Math.pow(ROBOT_WIDTH,2));
        double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double COUNTS_PER_INCH = PULSES_PER_REV * DRIVE_GEAR_REDUCTION/(Math.PI*WHEEL_DIAMETER);
        int COUNTS_PER_REV = PULSES_PER_REV*DRIVE_GEAR_REDUCTION;
        double NUM_REV_TO_90 = ROBOT_CIRCUMFERENCE/(4*WHEEL_CIRCUMFERENCE);
        int COUNTS_TO_90 = (int)Math.ceil(NUM_REV_TO_90*COUNTS_PER_REV);

        // Created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        String decision;
        int count;
        double speed;
        boolean turn90;

        public DriveThread(CompetitionHardware2 robot, int count, double speed, boolean turn90) {
            this.robot = robot;
            this.runtime = runtime;
            this.count = count;
            this.speed = speed;
            this.turn90 = turn90;
        }
        public void run() {
            try {
                Drive(this.count,this.speed,this.turn90);
            } catch (Exception e) {
                // Something bad happened
                //System.out.println(e.toString());
            }
        }

        private void Drive(int count, double speed, boolean turn90) {
            // Prepare to use encoders and reset them
            setModeToDrivetrain(DcMotor.RunMode.RUN_USING_ENCODER);
            resetAllEncoders();
            int currentFL;
            int currentFR;
            int currentRR;
            int currentRL;
            int posCount = count;
            int negCount = -count;
            double leftSpeed = speed;
            double rightSpeed = speed;
            if (speed < 0) {
                if (!turn90) {
                    // Reverse: Power stays the same; In this direction, the counts are positive
                    currentFL = robot.FLmotor.getCurrentPosition()+posCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+posCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+posCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+posCount;
                } else {
                    // Clockwise: Right is backward (negative) and left is forward (positive); In
                    // this direction, right should be positive counts and left should be negative.
                    rightSpeed = -rightSpeed;
                    posCount = COUNTS_TO_90;
                    negCount = -COUNTS_TO_90;
                    currentFL = robot.FLmotor.getCurrentPosition()+negCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+posCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+posCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+negCount;
                }
            } else {
                if (!turn90) {
                    // Forward: Power stays the same; In this direction, counts are negative.
                    currentFL = robot.FLmotor.getCurrentPosition()+negCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+negCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+negCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+negCount;
                } else {
                    // Counterclockwise: Left goes reverse (negative), and right goes forward (positive).;
                    // In this directio, left counts are positive and right counts are negative.
                    leftSpeed = -leftSpeed;
                    posCount = COUNTS_TO_90;
                    negCount = -COUNTS_TO_90;
                    currentFL = robot.FLmotor.getCurrentPosition()+posCount;
                    currentFR = robot.FRmotor.getCurrentPosition()+negCount;
                    currentRR = robot.RRmotor.getCurrentPosition()+negCount;
                    currentRL = robot.RLmotor.getCurrentPosition()+posCount;
                }
            }

            setModeToDrivetrain(DcMotor.RunMode.RUN_TO_POSITION);

            setTargetPositionOfDrivetrain(currentFL,currentRL,currentFR,currentRR);

            setPowerToDrivetrain(leftSpeed,rightSpeed);

            while (!isInterrupted() && robot.FLmotor.isBusy() && robot.FRmotor.isBusy() &&
                    robot.RLmotor.isBusy() && robot.RRmotor.isBusy()){
                //busy waiting
            }

            setPowerToDrivetrain(0,0);

            resetAllEncoders();

            // Assume you're going back to running manual
            setModeToDrivetrain(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        private void setModeToDrivetrain(DcMotor.RunMode mode) {
            robot.FLmotor.setMode(mode);
            robot.FRmotor.setMode(mode);
            robot.RLmotor.setMode(mode);
            robot.RRmotor.setMode(mode);
        }

        private void setPowerToDrivetrain(double leftPower, double rightPower) {
            robot.FLmotor.setPower(leftPower);
            robot.RLmotor.setPower(leftPower);
            robot.FRmotor.setPower(rightPower);
            robot.RRmotor.setPower(rightPower);
        }

        private void setTargetPositionOfDrivetrain(int FL, int RL, int FR, int RR) {
            robot.FLmotor.setTargetPosition(FL);
            robot.FRmotor.setTargetPosition(FR);
            robot.RLmotor.setTargetPosition(RL);
            robot.RRmotor.setTargetPosition(RR);
        }

        private void resetAllEncoders() {
            robot.FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private static class ElevatorThread extends Thread {

        int ELEVATOR_MAX = 5700;
        int COUNTS_TO_NEXT_MODE = (int) Math.ceil(ELEVATOR_MAX / 2);

        //created when needed
        CompetitionHardware2 robot;
        ElapsedTime runtime;
        double speed;
        boolean home;

        public ElevatorThread(CompetitionHardware2 robot, double speed, boolean home) {
            this.robot = robot;
            this.runtime = runtime;
            this.speed = speed;
            this.home = home;
        }

        public void run() {
            try {
                raiseLower(speed, home);
            } catch (Exception e) {
                //baddie baddie oh no.
            }
        }

        private void raiseLower(double speed, boolean home) {
            int NewPos;
            // I've taken telemetries out due to them not being static? if you ask me ¯\_ツ_/¯
            // (probably makes sense tho, Brandon is sleepy Zzz...)
            if (robot.elevatorMode < 0) {
                // Moving from manual to mode select (simple---Go Home first)
                robot.elevatorMode = 0;
                NewPos = 0;
            } else if (speed < 0 && robot.elevatorMode < 2 && !home) {
                // set the elevator to move upward
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur - COUNTS_TO_NEXT_MODE;
                robot.elevatorMode++;
            } else if (speed > 0 && robot.elevatorMode > 0 && !home) {
                //sets the elevator to move downward
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur + COUNTS_TO_NEXT_MODE;
                robot.elevatorMode--;
            } else if (home) {
                NewPos = 0;
                robot.elevatorMode = 0;
            } else {
                int cur = robot.Elevator.getCurrentPosition();
                NewPos = cur;
            }

            robot.Elevator.setTargetPosition(NewPos);

            robot.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.Elevator.setPower(speed);

            while(!isInterrupted() && robot.Elevator.isBusy()){
                if (robot.tBot.isPressed() && speed > 0) {
                    break;
                }
                if (robot.Elevator.getCurrentPosition() < -ELEVATOR_MAX && speed < 0) {
                    break;
                }
            }

            if (robot.tBot.isPressed()) {
                robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            robot.Elevator.setPower(0);
            robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
