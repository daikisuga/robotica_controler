/*
 * File:          Controlador.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>  // Dica para saber a posição da caixa

/*
 * You may want to add macros here.
 */

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */




int main(int argc, char **argv) {

  int i = 0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;

  /* Necessário para inicializar o Webots */
  wb_robot_init();

  // Caixa a ser 'estudada'
  
  WbNodeRef caixa_1 = wb_supervisor_node_get_from_def("caixa_1");
  WbNodeRef caixa_2 = wb_supervisor_node_get_from_def("caixa_2");
  WbNodeRef caixa_3 = wb_supervisor_node_get_from_def("caixa_3");
  WbNodeRef caixa_4 = wb_supervisor_node_get_from_def("caixa_4");
  WbNodeRef caixa_5 = wb_supervisor_node_get_from_def("caixa_5");
  WbNodeRef caixa_6 = wb_supervisor_node_get_from_def("caixa_6");
 
  WbNodeRef Epuck = wb_supervisor_node_get_from_def("robo");
 
  // Configurei MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  // Configurei Sensores de Proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];
  SensorProx[0] = wb_robot_get_device("ps0");
  SensorProx[1] = wb_robot_get_device("ps1");
  SensorProx[2] = wb_robot_get_device("ps2");
  SensorProx[3] = wb_robot_get_device("ps3");
  SensorProx[4] = wb_robot_get_device("ps4");
  SensorProx[5] = wb_robot_get_device("ps5");
  SensorProx[6] = wb_robot_get_device("ps6");
  SensorProx[7] = wb_robot_get_device("ps7");

  wb_distance_sensor_enable(SensorProx[0], TIME_STEP); //frente direita
  wb_distance_sensor_enable(SensorProx[1], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[2], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[3], TIME_STEP);// atras direito
  wb_distance_sensor_enable(SensorProx[4], TIME_STEP);// atras direito
  wb_distance_sensor_enable(SensorProx[5], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[6], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[7], TIME_STEP); //frente esquerda

  // Configura LEDs
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);

  /*
   * Main loop:
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    // Limpa a variável texto
    for (i = 0; i < 256; i++) texto[i] = 0;

    /*
     * Read the sensors:
     * Enter here functions to read sensor data, like:
     * double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Processa os dados dos sensores */
    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
    }
    printf("%s\n", texto);

    const double* posicoes[6];

    // Armazenando as posições das caixas no array
    posicoes[0] = wb_supervisor_node_get_position(caixa_1);
    posicoes[1] = wb_supervisor_node_get_position(caixa_2);
    posicoes[2] = wb_supervisor_node_get_position(caixa_3);
    posicoes[3] = wb_supervisor_node_get_position(caixa_4);
    posicoes[4] = wb_supervisor_node_get_position(caixa_5);
    posicoes[5] = wb_supervisor_node_get_position(caixa_6);
   
    //Posicao do robo
    const double *posicao_robo = wb_supervisor_node_get_position(Epuck);
    for (int j = 0; j < 6; j++) {
          printf("Posicao Caixa %d: %.2f, %.2f, %.2f\n", j + 1, posicoes[j][0], posicoes[j][1], posicoes[j][2]);
      }
    
    // Alterna o estado do LED
    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);
    
    
    /*
    Robo setado para começar olhando para SUL (240 graus)
    
    */
    
    //Primeiro vai atras da caixa 1
    if(LeituraSensorProx[0] > 50){//colidiu com algo
      double posicao antiga = posicao_robo;
      while(wb_robot_step(TIME_STEP) != -1){
    } 
    
    /*
     * Processamento de controles
     */
     
     
    if (LeituraSensorProx[0] > 30 || LeituraSensorProx[7] > 30) {
      
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
    } else {
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
    }

    // Configura a velocidade dos motores
    wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
