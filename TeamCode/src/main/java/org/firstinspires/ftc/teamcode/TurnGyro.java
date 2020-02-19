package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="GyroExample")
public class TurnGyro extends LinearOpMode {

    // Motors
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor Pivot;

    // Has to be configured as a "REV Expansion Hub IMU" in I2C Port 0 of either REV hub, named "imu"
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        // Hardware mapping
        frontleft = hardwareMap.dcMotor.get("FrontLeft");
        frontright = hardwareMap.dcMotor.get("FrontRight");
        backleft = hardwareMap.dcMotor.get("BackLeft");
        backright = hardwareMap.dcMotor.get("BackRight");

        imu             = hardwareMap.get(BNO055IMU.class, "imu");

        //
        CalibrateGyro();

        // Wait for the driver to press play
        waitForStart();

        // Turn ninety degrees to the left
        Turn(90);
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
}