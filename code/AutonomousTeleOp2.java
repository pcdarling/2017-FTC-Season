package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Prestolie on 11/3/2017.
 */

@Disabled
@Autonomous(name="autonomous2", group= "TeamBot")
public class AutonomousTeleOp2 extends LinearOpMode {

    public CompetitionHardware2 robot = new CompetitionHardware2();
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;

    // Preference Constants
    double STANDARD_SPEED = 0.5;
    double STANDARD_ELEVATOR_SPEED = 1;
    double STANDARD_SHOULDER_SPEED = 0.25;
    double STANDARD_CLAW_SPEED = 0.25;
    double LOWER_SHOULDER_SPEED = 0.05;
    double INC_SERVO = 0.1;
    double Y_DOWN = 0.1;
    double X_BACK = 0.4;
    int INC_SHOULDER = 1;
    int INC_ELEVATOR = 1;
    int INC_EXTENDER = 1;
    int OPEN_POS_NABBER = 175;
    int CLOSE_POS_NABBER = 140;
    int OPEN_CLEFT = 0;
    int OPEN_CRIGHT = 0;
    int CLOSE_CLEFT = 268; // Maybe 60?
    int CLOSE_CRIGHT = -268;
    int Y_HOME = 1;
    int X_HOME = 0;
    int GEAR_ERROR = -2;
    boolean SMOOTH = true;
    String TEAM = "blue";

    // Measured Constants
    double WHEEL_DIAMETER = 1.25; // inches, this was 4.875
    double ROBOT_LENGTH = 17.75;
    double ROBOT_WIDTH = 13.5;
    int COUNTS_TO_HORIZONTAL = 900;
    int COUNTS_TO_FIRST_LEVEL = 100;
    int COUNTS_TO_FULLY_EXTENDED = 2200;
    int ELEVATOR_MAX = 2200;
    int INCHES_TO_FIRST = (int)(30*2.54); // inches to centimeters
    int INCHES_TO_SECOND = (int)(36*2.54); // inches to centimeters
    int INCHES_TO_THIRD = (int)(42*2.54); // inches to centimeters

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
    double NUM_REV_TO_90 = NUM_REV_TO_180/2;

    double INCHES_PER_REV = Math.PI*WHEEL_DIAMETER; //15.7
    int COUNTS_PER_REV = PULSES_PER_REV * DRIVE_GEAR_REDUCTION;//280
    int COUNTS_TO_180 = (int)Math.ceil(NUM_REV_TO_180*COUNTS_PER_REV/2);
    int COUNTS_TO_90 = COUNTS_TO_180/2;
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
    int robotDirection;

    //Vuforia Variables
    int vuMarkID;

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        // set up Vuforia
        int camerMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(camerMonitorViewId);
        parameters.vuforiaLicenseKey = "AcV+Jsr/////AAAAGXHHw0GRIUwCpyZYG3N75VorKFs2D5Mw0kS8wVa0vpU47CzS6YPGhc3P1C4CUr4N4Qhio5ug/8GrmhBgAxK6AtikaT32qg3RXmC8D5jA7qn3gdQblkCxnLhH85YGcOA81URmFaK9ubqN8noR/08eR43sik1p+wdsyJgqOUHgEEs9CsYxJd3cm7IPj/wlUlEpa21R+KG/KFtnz9nNV6u5Ckgl5EztpBxur1dOabC1yxK5A7tlxQfbQJZSO0uz+hEOmvK5h3dUZwb7qPHb07MStcmTW+TkOkZSTJBN4VYEIFgY8yxqLnpEwz6uEZqoJrf1WmD49RR33UoTq+Pl58IJxJ1RvwVSz+9dvt1rXt3MHnM2"; //380 character license key, how about it not go across the screen?
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        setupRobot();

        waitForStart();
        relicTrackables.activate();
        // After pressing start, robot needs to start doing stuff. I've outlined where each part of
        // algorithm should go with telemetry statements
        long milliseconds = 3000;

        openClose("close", STANDARD_CLAW_SPEED);
        telemetry.addData("Algorithm Status:", "Closing Claw...");
        telemetry.update();

        raiseLower("up", STANDARD_ELEVATOR_SPEED);
        telemetry.addData("Algorithm Status:", "Raising Elevator...");
        telemetry.update();

        scanKey();
        telemetry.addData("Algorithm Status:","Reading cryptokey...");
        telemetry.update();

        knockGem(1);
        telemetry.addData("Algorithm Status:","Knocking over gem...");
        telemetry.update();
        sleep(milliseconds);

        goPark(vuMarkID);
        telemetry.addData("Algorithm Status:","Grabbing block and moving to cryptobox...");
        telemetry.update();

        sleep(milliseconds);
        placeCube();// does nothing ATM; just a placeholder.
        telemetry.addData("Algorithm Status:","Placing block..");
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

            int n = 1;
            while (opModeIsActive() && (robot.FLmotor.isBusy() && robot.FRmotor.isBusy() &&
                    robot.RLmotor.isBusy() && robot.RRmotor.isBusy())){
                // busy waiting
                if (SMOOTH == true) {    // as of now this would only really work if the robot is going forward or backward
                    //if FlMotor is halfway to its TargetPos
                    if (robot.FLmotor.getCurrentPosition() >= currentFL / 2) {
                        //will reduce the amount of speed every x amount of counts
                        if (robot.FLmotor.getPower() > 0.3){
                            n = 0;
                        }
                        else if (robot.FLmotor.getCurrentPosition() - n * 100 >= currentFL / 2) {
                            // will reduce the amount of power to 70% of its current speed(considering lower %)
                            setPowerToDrivetrain(speed * 0.7);
                            n += 1;
                        }
                    }
                }
            }

            setPowerToDrivetrain(0);

            setModeToDrivetrain(DcMotor.RunMode.RUN_USING_ENCODER);

            resetAllEncoders();
        }
    }

    public void rotate(double power) {
        if(power<0){
            setDirectionOfDriveTrain("counterclockwise");
        } else {
            setDirectionOfDriveTrain("clockwise");
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
        int NewPos;
        if (decision.equals("close")){
            //does incremental close
            robot.Cleft.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.Cright.setDirection(DcMotorSimple.Direction.REVERSE);
            double curLeft = robot.Cleft.getCurrentPosition();
            double curRight = robot.Cright.getCurrentPosition();
            robot.Cleft.setTargetPosition(CLOSE_CLEFT);
            robot.Cright.setTargetPosition(CLOSE_CRIGHT);
            telemetry.addData("Claw","Closing");
            telemetry.update();
        }
        else if (decision.equals("open")){
            //does incremental open
            robot.Cleft.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.Cright.setDirection(DcMotorSimple.Direction.REVERSE);
            double curLeft = robot.Cleft.getCurrentPosition();
            double curRight = robot.Cright.getCurrentPosition();
            robot.Cleft.setTargetPosition(OPEN_CLEFT);
            robot.Cright.setTargetPosition(OPEN_CRIGHT);
            telemetry.addData("Claw","Opening");
            telemetry.update();
        }
        else{
            robot.Cleft.setTargetPosition(OPEN_CLEFT);
            robot.Cright.setTargetPosition(OPEN_CRIGHT);
            telemetry.addData("Claw:", "Sumting Wong");
            telemetry.update();
        }
        robot.Cleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Cright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && robot.Cleft.isBusy() && robot.Cright.isBusy()){
            // busy waiting, also put an acceleration/deceleration function here.
            //i.e. thresholds met, decelerate as it gets closer to TargetPos
        }
        robot.Cleft.setPower(0);
        robot.Cright.setPower(0);

        robot.Cleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Cright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void knockGem(double GemState){
        boolean xMovement = false;

        if (GemState == 1){
            //sets down the gem stick
            robot.gemY.setPosition(Y_DOWN);
            GemState = 2;
        }
        if (GemState == 2) {
            runtime.reset();
            //checks for blue ball
            if (TEAM.equals("red")){

            }
                while(opModeIsActive() && runtime.seconds() < 2 && GemState == 2){
                    if (TEAM.equals("blue")) {
                        if (robot.Gem.blue() >= 2) {
                            robot.gemX.setPosition(X_BACK);
                            xMovement = true;
                            GemState = 3;
                        }
                    }
                    else{
                        if (TEAM.equals("red")){
                            if (robot.Gem.red() >= 2){
                                robot.gemX.setPosition(X_BACK);
                                xMovement = true;
                                GemState = 3;
                            }
                        }
                    }
                }
                //gemX will do nothing
                GemState = 3;
        }
        if (GemState == 3) {
            sleep(750);
            if (xMovement == true) {
                //sets the x-axis back to the home position
                robot.gemX.setPosition(X_HOME);
                xMovement = false;
                //then takes y back to home position
                robot.gemY.setPosition(Y_HOME);
                GemState = 0;
            } else {
                // move the robot for x amount of inches
                robot.gemY.setPosition(Y_HOME);
                GemState = 4;
            }
        }
        //lifts the gem stick back to the home position on y axis
        if (GemState == 4){
            sleep(750);
            robot.gemY.setPosition(Y_HOME);
        }

        while (GemState > 0){
            telemetry.addData("Algorithm Status:","Knocking over gem.");
            telemetry.update();
            idle();
        }
    }

    public int scanKey(){
        int camerMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(camerMonitorViewId);
        parameters.vuforiaLicenseKey = "AcV+Jsr/////AAAAGXHHw0GRIUwCpyZYG3N75VorKFs2D5Mw0kS8wVa0vpU47CzS6YPGhc3P1C4CUr4N4Qhio5ug/8GrmhBgAxK6AtikaT32qg3RXmC8D5jA7qn3gdQblkCxnLhH85YGcOA81URmFaK9ubqN8noR/08eR43sik1p+wdsyJgqOUHgEEs9CsYxJd3cm7IPj/wlUlEpa21R+KG/KFtnz9nNV6u5Ckgl5EztpBxur1dOabC1yxK5A7tlxQfbQJZSO0uz+hEOmvK5h3dUZwb7qPHb07MStcmTW+TkOkZSTJBN4VYEIFgY8yxqLnpEwz6uEZqoJrf1WmD49RR33UoTq+Pl58IJxJ1RvwVSz+9dvt1rXt3MHnM2";//380 character license key, how about not having it go across the screen?
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3 && vuMarkID == 0) {
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                vuMarkID = 1;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                vuMarkID = 2;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                vuMarkID = 3;
            } else {
                vuMarkID = 0;
            }
            if (vuMarkID != 0){
                return vuMarkID;
            }
        }
        return vuMarkID;
    }
    //the following function is not complete, only gets to the point where it faces cubes
    public void goPark(int vuMarkID) {
        int robotState = 1;
        // first, the robot will drive towards the parking zone before it makes a turn toward the pile of cubes
        if (robotState == 1){
        if (vuMarkID == 1) {
            //drive the robot the furthest distance, reaching the left side of the cryptobox
            telemetry.addData("VuMark:", "Left Detected");
            // the following drive counts are placeholders/estimates
            Drive(11, 0.4, false);
            sleep(1000);

            setPowerToDrivetrain(0);
            robotState = 2;
        } else if (vuMarkID == 2) {
            //drive the robot towards the middle of the cryptobox
            telemetry.addData("VuMark:", "Middle Detected");
            Drive(8, 0.4, false);
            sleep(1000);
            Drive(0, 0.4, true);//jk turns 90 degrees(ReKt)
            setPowerToDrivetrain(0);
            robotState = 2;
        } else if (vuMarkID == 3) {
            //drive the robot the shortest distance before turning
            telemetry.addData("VuMark:", "Right Detected");
            Drive(5, 0.4, false);
            sleep(1000);
            Drive(0, 0.4, true);//jk turns 90 degrees(ReKt)
            setPowerToDrivetrain(0);
            robotState = 2;
        } else {
            //this would be the situation where the cryptokey wasn't detected
            telemetry.addData("Vumark:", "Not Detected");
            Drive(8, 0.4, false);
            rotate(-0.4);
            sleep(1000);
            Drive(0, 0.4, true);//jk turns 90 degrees(ReKt)
            setPowerToDrivetrain(0);
            robotState = 2;
        }
    }
    }

    public void placeCube(){

    }

    // Below this point are "helper functions" meaning that they don't do anything special for
    // controlling the robot. They just do some functions that we use a lot.
    public void setDirectionOfDriveTrain(String direction) {
        if (direction.equals("counterclockwise")) {
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction.equals("clockwise")) {
            robot.FLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction.equals("reverse")) {
            robot.FLmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FRmotor.setDirection(DcMotor.Direction.FORWARD);
            robot.RLmotor.setDirection(DcMotor.Direction.REVERSE);
            robot.RRmotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction.equals("forward")) {
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
        runtime.reset();
        robot.FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.gemX.setPosition(X_HOME);
        robot.gemY.setPosition(Y_HOME);

        resetAllEncoders();

        //robot.Nabber.setPosition(OPEN_POS_NABBER);
        openClose("open",INC_SERVO);
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
        robot.gemX.setPosition(X_HOME);
        robot.gemY.setPosition(Y_HOME);
    }

}

