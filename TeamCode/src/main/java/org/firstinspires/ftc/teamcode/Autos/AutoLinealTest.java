package org.firstinspires.ftc.teamcode.Autos;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto IMU by Raúl", group = "Tests")
public class AutoLinealTest extends LinearOpMode {

    //To-Do
    //Tomar el angulo inicial de la IMU y utilizarlo para el resto del codigo (sumar y restar al valor inicial)

    //Variables globales
    DcMotor leftDrive, rightDrive;

    DcMotorEx shooterR, shooterL;

    CRServo intakeServo1, intakeServo2, chooserServoR, chooserServoL, regulationServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //Creación de motores e IMU
        leftDrive = hardwareMap.get(DcMotor.class, "ch1");
        rightDrive = hardwareMap.get(DcMotor.class, "ch2");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //Break or coast for motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");

        //Motor variable creation
        shooterR = hardwareMap.get(DcMotorEx.class, "d1");
        shooterL = hardwareMap.get(DcMotorEx.class, "d2");

        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Motor direction configuration
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Creacion de servo
        intakeServo1 = hardwareMap.get(CRServo.class, "in1"); //Primera llanta elastica
        intakeServo2 = hardwareMap.get(CRServo.class, "in2"); //Segunda llanta elastica
        chooserServoR = hardwareMap.get(CRServo.class, "a2"); // Llanta roja diamante derecha
        chooserServoL = hardwareMap.get(CRServo.class, "a1"); //Llanta roja diamante izquierda
        regulationServo = hardwareMap.get(CRServo.class, "x"); //Llanta negra

        // Servo direction configuration
        intakeServo1.setDirection(CRServo.Direction.REVERSE);
        intakeServo2.setDirection(CRServo.Direction.REVERSE);
        chooserServoR.setDirection(CRServo.Direction.FORWARD);
        chooserServoL.setDirection(CRServo.Direction.REVERSE);
        regulationServo.setDirection(CRServo.Direction.REVERSE);

        // Power variables
        double intakeServoPower = 0;
        double chooserServoPowerR = 0;
        double chooserServoPowerL = 0;
        double shooterPower = 0;
        double regulatorServoPower = 0;

        //Configuración para que los motores funcionen con encoder
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Heading1", obtenerAngulo());
            esperarAuto(3);
            giroAngulo(0);
            esperarAuto(3);
            giroAngulo(90);
            esperarAuto(4);
            giroAngulo(0);
            esperarAuto(4);
            giroAngulo(270);
            esperarAuto(4);
            giroAngulo(180);
            esperarAuto(2);
            break;
        }
    }

    // Creación de funciones adicionales //


    public void esperarAuto(double seconds) {
        double currentTime = getRuntime();
        while (getRuntime() < currentTime + seconds) {
            sleep(0);
            telemetry.addData("HEADING", obtenerAngulo());
            telemetry.update();
        }
    }

    //Función para giro preciso
    public void giroAngulo(double angulo) {
        double kp = 0.008;       // Ganancia proporcional (ajustable)
        double minPower = 0.07;  // Potencia mínima para vencer fricción estática
        double tolerancia = 0.3; // Error aceptable en grados

        while (opModeIsActive()) {
            double error = obtenerError(angulo);

            // Si el error es suficientemente pequeño, terminamos el giro
            if (Math.abs(error) <= tolerancia) break;

            // Calcular potencia proporcional
            double power = kp * error;

            // Asegurarse de que la potencia no sea demasiado baja
            if (Math.abs(power) < minPower) {
                power = minPower * Math.signum(error);
            }

            // Aplicar giro
            leftDrive.setPower(power);
            rightDrive.setPower(-power);

            telemetry.addData("Ángulo actual", obtenerAngulo());
            telemetry.addData("Error", error);
            telemetry.addData("Potencia", power);
            telemetry.update();
        }

        // Detener motores al final del giro
        poderMotor(0);
    }



    //Función para obtener ángulo
    public double obtenerAngulo() {
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }


    //Funcionar para calcular error (posición objetivo - posición actual)
    public double obtenerError(double anguloObjetivo) {
        //Calcular el error (distancia al ángulo objetivo)
        double error = anguloObjetivo - obtenerAngulo();
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    //Función para avanzar recto
    public void avanzarRecto(double poder, double distanciaCM) {
        double targetAngle = obtenerAngulo();
        double kp = 0.015;
        // Calcular ticks necesarios (ajusta según tus motores y ruedas)
        double ticksPorVuelta = 280;
        double diametroRuedaCM = 9.2;
        double ticksPorCM = ticksPorVuelta / (Math.PI * diametroRuedaCM); //28.88
        int ticksObjetivo = (int) (distanciaCM * ticksPorCM);

        // Resetear encoders y configurar modo
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(ticksObjetivo);
        rightDrive.setTargetPosition(ticksObjetivo);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(poder);
        rightDrive.setPower(poder);

        leftDrive.getCurrentPosition();
        leftDrive.getDirection();
        rightDrive.getDirection();

        while (opModeIsActive() &&
                Math.abs(leftDrive.getCurrentPosition()) < ticksObjetivo &&
                Math.abs(rightDrive.getCurrentPosition()) < ticksObjetivo) {

            double error = obtenerError(targetAngle);
            double correction = error * kp;

            // Limitar la corrección para evitar sobresaturación
            correction = Math.max(Math.min(correction, 0.3), -0.3);

            // Aplicar corrección (izquierda o derecha)
            leftDrive.setPower(poder + correction);
            rightDrive.setPower(poder - correction);

            telemetry.addData("Angulo", leftDrive.getCurrentPosition());
            telemetry.addData("DireccionMotorIzquierdo", leftDrive.getDirection());
            telemetry.addData("DireccionMotorDerecho", rightDrive.getDirection());
            telemetry.addData("Error", error);
            telemetry.addData("Corrección", correction);
            telemetry.addData("Heading: ", obtenerAngulo());
            telemetry.addData("L", leftDrive.getCurrentPosition());
            telemetry.addData("R", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        poderMotor(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Funcion de intake
    public void intake(double poderIntake) {
        intakeServo1.setPower(poderIntake);
        intakeServo2.setPower(poderIntake);
    }

    public void shooter(double shooterPower,double chooserServoPowerR, double chooserServoPowerL) {
        double regulatorServoPower = 0;

        double shooterVelocityR = shooterR.getVelocity();
        double shooterVelocityL = shooterL.getVelocity();
        double soltureR = shooterR.getVelocity();
        double soltureL = shooterL.getVelocity();
        double diference = soltureL - soltureR;

        shooterR.setPower(shooterPower);
        shooterL.setPower(shooterPower);
        chooserServoL.setPower(chooserServoPowerL);
        chooserServoR.setPower(chooserServoPowerR);

        regulationServo.setPower(regulatorServoPower);

        if(shooterVelocityL > 1350 && shooterVelocityR > 1350 && diference < 10 && diference > -10){
            regulatorServoPower = 0.8;
        }

        //Falta telemetria
        telemetry.addData("Poder llanta Negra", regulatorServoPower);
        telemetry.addData("Poder shooters", shooterPower);
        telemetry.addData("Poder Chooser Servo Derecho", chooserServoPowerR);
        telemetry.addData("Poder Chooser Servo Izquierdo", chooserServoPowerL);
        telemetry.addData("Velocidad shooter derecho", shooterVelocityR);
        telemetry.addData("Velocidad shooter izquierdo", shooterVelocityL);
        telemetry.update();
    }

    //Funciones de giro
    public void enfrente(double poder) {
        leftDrive.setPower(poder);
        rightDrive.setPower(poder);
    }

    public void atras(double poder) {
        leftDrive.setPower(-poder);
        rightDrive.setPower(-poder);
    }

    public void derecha(double poder) {
        leftDrive.setPower(poder);
        rightDrive.setPower(-poder);
    }

    public void izquierda(double poder) {
        leftDrive.setPower(-poder);
        rightDrive.setPower(poder);
    }

    public void poderMotor(double poder) {
        leftDrive.setPower(poder);
        rightDrive.setPower(poder);
    }
}