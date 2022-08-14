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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Derived from RobotAutoDriveByTime_Linear 14-AUG-2022 by Joe Levy
 * Second in a series of "Drive Straight" demo OpModes.  Incorporates BNO055 Orientation Sensor
 * feedback for directional control.
 *
 *
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Drive With Feedback", group="Robot")
//@Disabled
public class DriveStraightFb2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.3;    // Fraction of full motor power
    static final double     RUN_TIME = 5.0;         // Total drive time in seconds


    // IMU variables, objects, initialization code all copied from SensorBNO055IMU

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Variables we're going to use for steering
    double bearing;     // The target angle returned by the IMU before we start moving
    double heading;     // The measured angle read from the IMU during the drive
    double error;       // Calculated difference between bearing and heading
    double powerAdjustment;     // Increase & decrease wheel speed by this amount (calculated from error, K_PROP)

    static final double K_PROP = 0.05 * FORWARD_SPEED;      // Proportionality constant for PID control

    @Override
    public void runOpMode() {

        // More IMU stuff copied from SensorBNO055IMU

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "lbm");     // Read from DefenseBot config
        rightDrive = hardwareMap.get(DcMotor.class, "rfm");     // Read from DefenseBot config

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        sleep(1000);

        while(opModeInInit()){      // TRYING NEW METHOD AVAILABLE IN SDK 7.2
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Current Heading: ", angles.firstAngle);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();    // Start the clock

        // get the initial bearing, which is wherever the robot was pointing at "play".
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bearing = angles.firstAngle;    // Again, using SensorBNO055IMU as an example
        heading = bearing;              // initialization
        error = bearing - heading;      // will be zero at initialization
        powerAdjustment = 0.0;          // will be zero at initialization

        /*
         * Discussion:
         *
         * Drift to the right (clockwise) will yield heading < bearing, therefore positive values
         * for error.  Corrective power adjustments should be an increase in power to the right wheel
         * and a decrease in power to the left wheel.  The converse will be true for drift to the
         * left (counter-clockwise).  We will keep the sign of powerAdjustment the same as error,
         * so powerAdjustment will always be added to the right wheel and subtracted from the left.
         *
         */

       /*
        * The main event.  Loop around for the duration of the drive cycle, making adjustment
        * as the control algorithm requires.
        *
        * Now, we introduce feedback to control drive power.
        *
        */
        while (opModeIsActive() && (runtime.seconds() < RUN_TIME)) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            error = bearing - heading;
            if(Math.abs(error)>25){     // Failsafe if we're more than 25 degrees off
                terminateOpModeNow();   // Opportunity to try new SDK method
            }
            powerAdjustment = error * K_PROP;   // Convert degrees of error to a motor power adjustment
            leftDrive.setPower(FORWARD_SPEED-powerAdjustment);
            rightDrive.setPower(FORWARD_SPEED+powerAdjustment);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
