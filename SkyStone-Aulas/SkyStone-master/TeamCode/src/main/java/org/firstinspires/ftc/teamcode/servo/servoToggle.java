package org.firstinspires.ftc.teamcode.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name=" Servo toggle", group="Linear Opmode")
public class servoToggle extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declaração dos Servos
    Servo servoDireita = null;
    Servo servoEsquerda = null;

    //Variáveis para toggle
    boolean antB = false;
    boolean toggle = true;

    @Override
    public void runOpMode() {
        //Variáveis gamepad
        boolean buttonA;

        //variável para posição do servo
        double servoPos = 0;

        //Vinculação ao Rc
        servoDireita = hardwareMap.get(Servo.class, "servo_Direita");
        servoEsquerda = hardwareMap.get(Servo.class, "servo_Esquerda");

        //Direção dos servos
        servoDireita.setDirection(Servo.Direction.FORWARD);
        servoEsquerda.setDirection(Servo.Direction.REVERSE);

        //Movimenta o servo quando inicia o robô para 0
        servoDireita.setPosition(servoPos);
        servoEsquerda.setPosition(servoPos);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Referenciando a variável buttonA ao botão a do gamepad
            buttonA = gamepad1.a;

            /* Este é um toggle tirado do reddit com uma miníma modificação nossa.
             * https://www.reddit.com/r/FTC/comments/3z75ou/use_one_button_to_onoff/
             * https://www.reddit.com/r/FTC/comments/enembp/programming_toggles/
             */
            if(buttonA && !antB) {
                //Operador ternário, mais prático que dois ou + if's
                servoPos = toggle ? 1 : 0;
                toggle = !toggle;
                antB = true;
            } else if(!buttonA) {
                antB = false;
            }

            //Manda o valor para os servos
            servoDireita.setPosition(servoPos);
            servoEsquerda.setPosition(servoPos);
        }

    }

}
