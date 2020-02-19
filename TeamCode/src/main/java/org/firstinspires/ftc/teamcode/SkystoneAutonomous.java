//SkystoneAutonomous.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Sample Red")
public class SkystoneAutonomous extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector = new SkystoneDetector();
    private String position;
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor Pivot;



    Servo RightPivot;
    Servo LeftPivot;
    Servo RightPinch;
    Servo LeftPinch;




    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .20, correction;

    //28 * 20 / (2ppi * 4.125)
    Double width = 17.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.9;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        Pivot = hardwareMap.dcMotor.get("Pivot");

        initGyro();
        //Motors
        frontleft = hardwareMap.dcMotor.get("FrontLeft");
        frontright = hardwareMap.dcMotor.get("FrontRight");
        backleft = hardwareMap.dcMotor.get("BackLeft");
        backright = hardwareMap.dcMotor.get("BackRight");

        //Servos

        RightPivot = hardwareMap.servo.get("RightPivot");
        LeftPivot = hardwareMap.servo.get("LeftPivot");

        RightPinch = hardwareMap.servo.get("RightPinch");
        LeftPinch = hardwareMap.servo.get("LeftPinch");








        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            //
            CalibrateGyro();
            telemetry.update();
        }




        // code while robot is running
        if (position.equals("LEFT")) {
            LeftPivot.setPosition(0.6);
            LeftPinch.setPosition(0.0);
            RightPinch.setPosition(0.0);
            sleep(300);

            strafeToPosition(-4,0.2);

            moveToPosition(23,0.2);

            Freeze(0);
            sleep(300);

            //LeftPivot.setPosition(0.4);
            //sleep(600);

            RightPivot.setPosition(0.75);
            sleep(300);

            moveToPosition(6,0.1);

            RightPinch.setPosition(1.0);
            sleep(500);

            Freeze(0);
            sleep(300);

            RightPivot.setPosition(0.3);
            sleep(400);

            moveToPosition(-4,0.2);

            Freeze(0);
            sleep(300);

            turnWithGyro(83,0.2);

            Freeze(0);
            sleep(300);

            //strafeToPosition(-2,0.2);

            moveToPosition(50,0.2);

            Freeze(0);
            sleep(300);

            RightPinch.setPosition(0.5);
            sleep(300);



            Freeze(0);
            sleep(300);

            moveToPosition(-18,0.2);

            Freeze(0);
            sleep(400);

            strafeToPosition(-5,0.2);

        }

        if (position.equals("CENTER")){

            LeftPivot.setPosition(0.6);
            LeftPinch.setPosition(0.0);
            RightPinch.setPosition(0.0);
            sleep(300);

            strafeToPosition(4,0.2);

            moveToPosition(23,0.2);

            Freeze(0);
            sleep(300);



            //LeftPivot.setPosition(0.4);
            //sleep(600);

            RightPivot.setPosition(0.75);
            sleep(300);

            moveToPosition(6,0.1);

            RightPinch.setPosition(1.0);
            sleep(500);

            Freeze(0);
            sleep(300);

            RightPivot.setPosition(0.3);
            sleep(400);

            moveToPosition(-4,0.2);

            Freeze(0);
            sleep(300);

            turnWithGyro(80,0.2);

            Freeze(0);
            sleep(300);

            //strafeToPosition(-2,0.2);

            moveToPosition(50,0.2);

            Freeze(0);
            sleep(300);

            RightPinch.setPosition(0.5);
            sleep(300);



            Freeze(0);
            sleep(300);

            moveToPosition(-18,0.2);

            Freeze(0);
            sleep(400);

            strafeToPosition(-5,0.2);

        }

        if (position.equals("RIGHT")){

            LeftPivot.setPosition(0.6);
            LeftPinch.setPosition(0.4);
            RightPinch.setPosition(0.0);
            sleep(300);

            sleep(300);

            strafeToPosition(-2,0.2);



            moveToPosition(23,0.2);

            Freeze(0);
            sleep(300);


            //LeftPivot.setPosition(0.4);
            //sleep(600);

            LeftPivot.setPosition(0.2);
            sleep(300);

            moveToPosition(6,0.1);

            LeftPinch.setPosition(0.0);
            sleep(500);

            Freeze(0);
            sleep(300);

            LeftPivot.setPosition(0.35);
            sleep(400);

            moveToPosition(-4,0.2);

            Freeze(0);
            sleep(300);

            //turnWithGyro(78,0.2);
            Turn(-90);

            Freeze(0);
            sleep(300);

            //strafeToPosition(-2,0.2);

            moveToPosition(50,0.2);

            Freeze(0);
            sleep(300);

            LeftPinch.setPosition(0.2);
            sleep(300);



            Freeze(0);
            sleep(300);

            moveToPosition(-15,0.2);

            Freeze(0);
            sleep(400);

            strafeToPosition(-5,0.2);

            /*

            Freeze(0);
            sleep(300);



            turnWithGyro(74,-0.3);

            Freeze(0);
            sleep(300);

            RightPivot.setPosition(0.75);
            sleep(300);

            moveToPosition(13,0.2);

            Freeze(0);
            sleep(300);

            RightPinch.setPosition(1.0);
            sleep(500);

            Freeze(0);
            sleep(300);

            RightPivot.setPosition(0.3);

            sleep(400);

            moveToPosition(-4,0.2);

            Freeze(0);
            sleep(300);

            turnWithGyro(83,0.2);

            Freeze(0);
            sleep(300);

            moveToPosition(60,0.3);

            Freeze(0);
            sleep(300);

            RightPinch.setPosition(0.5);

            moveToPosition(-15,0.3);

            PivotPower(-.7);
            sleep(2000);

             */





















        }
    }

    // Turning functions
    double MinTurnSpeed = 0.1;              // Robot won't turn slower than this
    double MaxTurnSpeed = 0.27;              // Robot won't turn faster than this
    double AccelAngle = 45;                 // The amount of degrees given to accelerate from the minimum speed to the maximum
    private void Turn(double degrees) {

        // Update the target angle
        TargetAngle = NormalizeAngle(TargetAngle + degrees);

        // Loops until the robot has reached the target
        while (opModeIsActive()) {

            // Calculate how many degrees the robot still has to turn
            double DegreesRemaining = NormalizeAngle(TargetAngle - GetRobotAngle());

            // Calculate the turn speed
            double TurnSpeed;
            if (DegreesRemaining < 180)
                TurnSpeed = -MinTurnSpeed + -((MaxTurnSpeed - MinTurnSpeed) * Math.min(DegreesRemaining / AccelAngle, 1));
            else
                TurnSpeed = MinTurnSpeed + ((MaxTurnSpeed - MinTurnSpeed) * Math.min((360 - DegreesRemaining) / AccelAngle, 1));

            // Apply the new motor speeds
            backleft.setPower(TurnSpeed);
            backright.setPower(TurnSpeed);
            frontleft.setPower(TurnSpeed);
            frontright.setPower(TurnSpeed);

            if(Math.abs(DegreesRemaining) < 1)
                break;
        }

        // Stop the motors
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }


    // IMU functions
    double RobotAngle = 0;
    double TargetAngle = 0;
    private void CalibrateGyro() {

        // Sets up the parameters and waits for it to initialize
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()){}
    }
    private double GetRobotAngle() {
        RobotAngle = NormalizeAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        return RobotAngle;
    }
    private double NormalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 8) {
            int move = (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move);
            backright.setTargetPosition(backright.getCurrentPosition() + move);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 8) * conversion));
            int movefl2 = frontleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movefr2 = frontright.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebl2 = backleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebr2 = backright.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move1);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move1);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move1);
            backright.setTargetPosition(backright.getCurrentPosition() + move1);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            //
            frontleft.setTargetPosition(movefl2);
            frontright.setTargetPosition(movefr2);
            backleft.setTargetPosition(movebl2);
            backright.setTargetPosition(movebr2);
            //
            frontleft.setPower(.1);
            frontright.setPower(.1);
            backleft.setPower(.1);
            backright.setPower(.1);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void PivotPower (double power) {

        Pivot.setPower(power);
    }


    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    // Gyro functions
    double lastAngle = 0;
    double currentAngle = 0;
    private double getRobotAngle() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        currentAngle = normalizeAngle(currentAngle + angle - lastAngle);
        lastAngle = angle;
        return currentAngle;
    }
    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0)
            angle += 360;
        return angle;
    }


    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }



    public void Freeze(double power){
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }
}

