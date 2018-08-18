package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Prestolie on 11/3/2017.
 */

@Disabled
@Autonomous(name="autonomous", group= "TeamBot")
public class AutonomousTeleOp extends LinearOpMode {

    public CompetitionHardware robot = new CompetitionHardware();

    // Preference Constants
    double STANDARD_SPEED = 0.5;
    double STANDARD_ELEVATOR_SPEED = 0.3;
    double STANDARD_SHOULDER_SPEED = 0.25;
    double LOWER_SHOULDER_SPEED = 0.05;
    double INC_SERVO = 0.1;
    int INC_SHOULDER = 1;
    int INC_ELEVATOR = 1;
    int INC_EXTENDER = 1;
    int OPEN_POS_NABBER = 175;
    int CLOSE_POS_NABBER = 140;
    int OPEN_CLEFT = 255;
    int OPEN_CRIGHT = 0;
    int CLOSE_CLEFT = 100; // Maybe 60?
    int CLOSE_CRIGHT = 120;
    int GEAR_ERROR = -2;

    // Measured Constants
    double WHEEL_DIAMETER = 1.25; // inches, this was 4.875
    double ROBOT_LENGTH = 17.75;
    double ROBOT_WIDTH = 13.5;
    int COUNTS_TO_HORIZONTAL = 900;
    int COUNTS_TO_FIRST_LEVEL = 100;
    int COUNTS_TO_FULLY_EXTENDED = 2200;
    int ELEVATOR_MAX = 2200;

    // Hardware Constants
    int DRIVE_GEAR_REDUCTION = (20*40/80+GEAR_ERROR); // Internal gearbox is 20:1 and external is 1:2, also, its not exact
    int PULSES_PER_REV = 28;
    double RETRACTED_LENGTH = 16; // inches
    double IDOL_HEIGHT = 10; // inches
    double GRABBER_REDUCTION = 1.5; // inches
    double WALL_HEIGHT = 12; // inches
    int SHOULDER_GEAR_REDUCTION = 30;
    int EXTENDER_GEAR_REDUCTION = 30;
    int ELEVATOR_GEAR_REDUCTION = 40;
    double DEGREES_PER_COUNT = 0.7059;
    double MAX_SERVO = 255;

    // Calculated Constants
    double ROBOT_DIAMETER = Math.sqrt(Math.pow(ROBOT_LENGTH,2) + Math.pow(ROBOT_WIDTH,2));
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
    double NUM_REV_TO_180 = ROBOT_CIRCUMFERENCE/(2 * WHEEL_CIRCUMFERENCE);

    double INCHES_PER_REV = Math.PI*WHEEL_DIAMETER; //15.7
    int COUNTS_PER_REV = PULSES_PER_REV * DRIVE_GEAR_REDUCTION;//280
    int COUNTS_TO_180 = (int)Math.ceil(NUM_REV_TO_180*COUNTS_PER_REV);
    int COUNTS_PER_INCH = (int) Math.ceil(COUNTS_PER_REV/INCHES_PER_REV); //18

    int COUNTS_PER_DEGREE = COUNTS_TO_HORIZONTAL/90;
    double HOLD_HEIGHT = (IDOL_HEIGHT-GRABBER_REDUCTION)+WALL_HEIGHT;
    double REQUIRED_ANGLE_FROM_HORIZONTAL = Math.asin(HOLD_HEIGHT/RETRACTED_LENGTH);
    int COUNTS_TO_OVER_WALL = (int)Math.ceil(REQUIRED_ANGLE_FROM_HORIZONTAL*COUNTS_PER_DEGREE);
    int COUNTS_TO_NEXT_LEVEL = (int)Math.ceil((COUNTS_TO_FULLY_EXTENDED-COUNTS_TO_FIRST_LEVEL)/3);
    int COUNTS_TO_NEXT_MODE = (int)Math.ceil(ELEVATOR_MAX/3);

    // State of machine
    int shoulderMode = 0;
    int extensionMode = 0;
    int elevatorMode = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        setupRobot();

        waitForStart();

        // After pressing start, robot needs to start doing stuff. I've outlined where each part of
        // algorithm should go with telemetry statements
        long milliseconds = 3000;

        telemetry.addData("Algorithm Status:","Knocking over gem.");
        telemetry.update();

        sleep(milliseconds);

        telemetry.addData("Algorithm Status:","Reading cryptokey.");
        telemetry.update();

        sleep(milliseconds);

        telemetry.addData("Algorithm Status:","Grabbing block and moving to cryptobox.");
        telemetry.update();

        sleep(milliseconds);

        telemetry.addData("Algorithm Status:","Placing block");
        telemetry.update();

        shutdownRobot();
    }

    public void Drive(int count, double speed, boolean turn180) {
        int leftCount = count;
        int rightCount = -count;
        if (speed < 0) {
            if (!turn180) {
                telemetry.addData("Status:", "Going forward");
                setDirectionOfDriveTrain("forward");
            } else {
                telemetry.addData("Status:", "Rotating Clockwise");
                setDirectionOfDriveTrain("clockwise");
                leftCount = COUNTS_TO_180;
                rightCount = -COUNTS_TO_180;
            }
        } else {
            if (!turn180) {
                telemetry.addData("Status:", "Going reverse");
                setDirectionOfDriveTrain("reverse");
            } else {
                telemetry.addData("Status:", "Rotating Counterclockwise");
                setDirectionOfDriveTrain("counterclockwise");
                leftCount = COUNTS_TO_180;
                rightCount = -COUNTS_TO_180;
            }
        }

        if (opModeIsActive()){
            int currentFL = robot.FLmotor.getCurrentPosition()+leftCount;
            int currentFR = robot.FRmotor.getCurrentPosition()+rightCount;
            int currentRR = robot.RRmotor.getCurrentPosition()+rightCount;
            int currentRL = robot.RLmotor.getCurrentPosition()+leftCount;

            setModeToDrivetrain(DcMotor.RunMode.RUN_TO_POSITION);

            setTargetPositionOfDrivetrain(currentFL,currentRL,currentFR,currentRR);

            setPowerToDrivetrain(speed);

            while (opModeIsActive() && (robot.FLmotor.isBusy() && robot.FRmotor.isBusy() &&
                    robot.RLmotor.isBusy() && robot.RRmotor.isBusy())){
                // busy waiting
                telemetry.addData("Encoder Count:",String.format("%d",robot.FRmotor.getCurrentPosition()));
                telemetry.addData("Encoder Count:",String.format("%d",robot.FLmotor.getCurrentPosition()));
                telemetry.addData("Encoder Count:",String.format("%d",robot.RRmotor.getCurrentPosition()));
                telemetry.addData("Encoder Count:",String.format("%d",robot.RLmotor.getCurrentPosition()));
                telemetry.addData("Desired Count:",String.format("%d",count));
                telemetry.update();
            }

            setPowerToDrivetrain(0);

            setModeToDrivetrain(DcMotor.RunMode.RUN_USING_ENCODER);

            resetAllEncoders();
        }
    }

    public void rotate(double power) {
        if(power<0){
            setDirectionOfDriveTrain("clockwise");
        } else {
            setDirectionOfDriveTrain("counterclockwise");
        }
        setPowerToDrivetrain(power);
    }

    public void rotateShoulder (String decision, double speed) {
        int newPos;
        if ((decision.equals("up") || decision.equals("down") || decision.equals("home")) && shoulderMode < 0){
            telemetry.addData("Crane:","Going home due to manual movement!");
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            newPos = 0;
            shoulderMode = 0;
            telemetry.addData("Shoulder:","Going Home due to manual movement.");
        } else if (decision.equals("up") && shoulderMode < 2 && shoulderMode >= 0) {
            // Rotate towards over-the-wall position
            telemetry.addData("Crane:","Heading towards raised position!");
            robot.Shoulder.setDirection(DcMotor.Direction.FORWARD);
            if (shoulderMode == 0) {
                // Encoder should be around 0
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_HORIZONTAL;
            } else {
                // Encoder should be around COUNTS_TO_HORIZONTAL
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_OVER_WALL;
            }
            shoulderMode++;
        } else if (decision.equals("down") && shoulderMode > 0) {
            // Rotate towards resting position
            telemetry.addData("Crane:","Heading towards resting position!");
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            if (shoulderMode == 2) {
                // Encoder should be around COUNTs_TO_HORIZONTAL+COUNTS_TO_OVER_WALL
                newPos = robot.Shoulder.getCurrentPosition() + COUNTS_TO_OVER_WALL;
            } else {
                // Encoder should be around COUNTS_TO_HORIZONTAL
                newPos = 0;
            }
            shoulderMode--;
        } else if (decision.equals("home")) {
            // Set the arm back to its resting position
            telemetry.addData("Crane:","Going home!");
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            newPos = 0;
            shoulderMode = 0;
        } else if (decision.equals("manual")){
            // Manually rotating shoulder
            telemetry.addData("Crane:","Going into manual movement!");
            shoulderMode = -1;
            if(speed < 0){
                robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                robot.Shoulder.setDirection(DcMotor.Direction.FORWARD);
            }
            newPos = robot.Shoulder.getCurrentPosition() + INC_SHOULDER;
        } else {
            telemetry.addData("Crane:","State machine says stay here!");
            int cur = robot.Elevator.getCurrentPosition();
            newPos = cur;
        }
        telemetry.update();

        robot.Shoulder.setTargetPosition(newPos);

        robot.Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        robot.Shoulder.setPower(speed);

        while(opModeIsActive() && robot.Shoulder.isBusy()){
            //busy waiting
            telemetry.addData("Crane Status:", "Rotating");
            telemetry.update();
        }

        robot.Shoulder.setPower(0);
        robot.Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*public void extendRetract(int decision, double speed){
        int newPos;
        if (decision == 0 && extensionMode < 3 && extensionMode >= 0) {
            // Extend Arm
            robot.Shoulder.setDirection(DcMotor.Direction.FORWARD);
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
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
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
            robot.Shoulder.setDirection(DcMotor.Direction.REVERSE);
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
    }*/

    /*public void grabRelease (int decision){
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
    }*/

    public void raiseLower(String decision, double speed) {
        int NewPos;

        if ((decision.equals("up") || decision.equals("down") || decision.equals("home")) && elevatorMode < 0) {
            // Moving from manual to mode select (simple---Go Home first)
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            int cur = robot.Elevator.getCurrentPosition();
            elevatorMode = 0;
            NewPos = 0;
            telemetry.addData("Elevator","Going Home due to Manual Movement");
            telemetry.update();
        }
        else if (decision.equals("up") && elevatorMode < 2){
            // set the elevator to move upward
            robot.Elevator.setDirection(DcMotor.Direction.FORWARD);
            int cur = robot.Elevator.getCurrentPosition();
            NewPos = cur+COUNTS_TO_NEXT_MODE;
            elevatorMode++;
            telemetry.addData("Elevator","Going Up!");
            telemetry.update();
        }
        else if (decision.equals("down") && elevatorMode  > 0){
            //sets the elevator to move downward
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            int cur = robot.Elevator.getCurrentPosition();
            NewPos = cur+COUNTS_TO_NEXT_MODE;
            elevatorMode--;
            telemetry.addData("Elevator","Going Down!");
            telemetry.update();
        }
        else if (decision.equals("home")){
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            int cur = robot.Elevator.getCurrentPosition();
            NewPos = 0;
            elevatorMode = 0;
            telemetry.addData("Elevator","Going Home!");
            telemetry.update();
        }
        else if (decision.equals("manual")){
            elevatorMode = -1;
            if(speed < 0){
                robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                robot.Elevator.setDirection(DcMotor.Direction.FORWARD);
            }
            int cur = robot.Elevator.getCurrentPosition();
            NewPos = robot.Elevator.getCurrentPosition() + INC_ELEVATOR;
            telemetry.addData("Elevator","Switching to manual control");
            telemetry.update();
        } else {
            telemetry.addData("Elevator","State machine says stay here!");
            telemetry.update();
            int cur = robot.Elevator.getCurrentPosition();
            NewPos = cur;
        }

        robot.Elevator.setTargetPosition(NewPos);

        robot.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        robot.Elevator.setPower(speed);

        while(opModeIsActive() && robot.Elevator.isBusy()){
            //busy waiting
            telemetry.addData("Elevator Status:", "Current Count = %d",robot.Elevator.getCurrentPosition());
            telemetry.addData("Target Position:",NewPos);
        }

        robot.Elevator.setPower(0);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openClose(String decision, double speed) {
        if (decision.equals("inc_close")){
            //does incremental close
            double curLeft = robot.Cleft.getPosition();
            double curRight = robot.Cright.getPosition();
            robot.Cleft.setPosition(curLeft - INC_SERVO);
            robot.Cright.setPosition(curRight + INC_SERVO);
            telemetry.addData("Claw","Incrementally Closing");
            telemetry.update();
        }
        else if (decision.equals("inc_open")){
            //does incremental open
            double curLeft = robot.Cleft.getPosition();
            double curRight = robot.Cright.getPosition();
            robot.Cleft.setPosition(curLeft + INC_SERVO);
            robot.Cright.setPosition(curRight - INC_SERVO);
            telemetry.addData("Claw","Incrementally Opening");
            telemetry.update();
        }
        else if (decision.equals("close")){
            //does closes
            robot.Cleft.setPosition(CLOSE_CLEFT/MAX_SERVO);
            robot.Cright.setPosition(CLOSE_CRIGHT/MAX_SERVO);
            telemetry.addData("Claw","Closing");
            telemetry.update();
        }
        else {
            //does opens
            robot.Cleft.setPosition(OPEN_CLEFT/MAX_SERVO);
            robot.Cright.setPosition(OPEN_CRIGHT/MAX_SERVO);
            telemetry.addData("Shoulder","Opening");
            telemetry.update();
        }
    }

    // Below this point are "helper functions" meaning that they don't do anything special for
    // controlling the robot. They just do some functions that we use a lot.
    public void setDirectionOfDriveTrain(String direction) {
        if (direction.equals("forward")) {
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction.equals("reverse")) {
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction.equals("clockwise")) {
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction.equals("counterclockwise")) {
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            // Do nothing since you put in something wrong
            telemetry.addData("Direction Error:","Invalid Command in code somewhere.");
            telemetry.update();
        }
    }

    public void setModeToDrivetrain(DcMotor.RunMode mode) {
        robot.FLmotor.setMode(mode);
        robot.FRmotor.setMode(mode);
        robot.RLmotor.setMode(mode);
        robot.RRmotor.setMode(mode);
    }

    public void setPowerToDrivetrain(double power) {
        power = Math.abs(power);
        robot.FLmotor.setPower(power);
        robot.RLmotor.setPower(power);
        robot.FRmotor.setPower(power);
        robot.RRmotor.setPower(power);
    }

    public void setTargetPositionOfDrivetrain(int FL, int RL, int FR, int RR) {
        robot.FLmotor.setTargetPosition(FL);
        robot.FRmotor.setTargetPosition(FR);
        robot.RLmotor.setTargetPosition(RL);
        robot.RRmotor.setTargetPosition(RR);
    }

    public void resetAllEncoders() {
        robot.FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setupRobot() {
        robot.FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetAllEncoders();

        //robot.Nabber.setPosition(OPEN_POS_NABBER);
        openClose("close",INC_SERVO);
    }

    public void shutdownRobot() {
        // Close the Nabber
        //robot.Nabber.setPosition(CLOSE_POS_NABBER);

        // Fully retract Extender
        //extendRetract(2,STANDARD_SPEED);

        // Fully lower Shoulder
        rotateShoulder("home",LOWER_SHOULDER_SPEED);

        // Open the claw
        openClose("open",INC_SERVO);

        // Fully retract the Elevator
        raiseLower("home",STANDARD_SPEED);
    }

}

