#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define NUM_SENSORS 8
#define MAX_SPEED 6.28
#define NUM_BOXES 20

int main() {
  int i;
  wb_robot_init();

  srand(time(NULL)); // Para movimentos aleatórios

  // Dispositivos
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  WbDeviceTag prox[NUM_SENSORS];
  char sensor_name[5];
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ps%d", i);
    prox[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(prox[i], TIME_STEP);
  }

  // Supervisor
  WbNodeRef self = wb_supervisor_node_get_self();

  // Caixas
  WbNodeRef caixas[NUM_BOXES];
  const double *caixa_pos_ant[NUM_BOXES];
  char def[16];
  for (i = 0; i < NUM_BOXES; i++) {
    sprintf(def, "CAIXA%d", i + 1);
    caixas[i] = wb_supervisor_node_get_from_def(def);
    caixa_pos_ant[i] = (caixas[i]) ? wb_supervisor_node_get_position(caixas[i]) : NULL;
  }

  // Movimento aleatório inicial
  double v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  double v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  wb_motor_set_velocity(left_motor, v_left);
  wb_motor_set_velocity(right_motor, v_right);

  bool colidiu = false;
  int timer = 100; // tempo andando aleatoriamente

  while (wb_robot_step(TIME_STEP) != -1) {
    // Lê sensores
    int bateu = 0;
    for (i = 0; i < NUM_SENSORS; i++) {
      double val = wb_distance_sensor_get_value(prox[i]);
      if (val > 80.0) bateu = 1;
    }

    if (!colidiu && bateu) {
      colidiu = true;
      printf("COLIDIU!\n");

      // Espera 1 segundo para caixa se mover
      int t;
      for (t = 0; t < 30; t++) wb_robot_step(TIME_STEP);

      for (i = 0; i < NUM_BOXES; i++) {
        if (!caixas[i] || !caixa_pos_ant[i]) continue;
        const double *nova_pos = wb_supervisor_node_get_position(caixas[i]);

        if (nova_pos && (
            fabs(nova_pos[0] - caixa_pos_ant[i][0]) > 0.01 ||
            fabs(nova_pos[2] - caixa_pos_ant[i][2]) > 0.01)) {
          // A caixa se moveu
          printf("Caixa %d se moveu! DANÇA!\n", i + 1);

          // Dança: gira no próprio eixo
          for (int j = 0; j < 100; j++) {
            wb_motor_set_velocity(left_motor, MAX_SPEED);
            wb_motor_set_velocity(right_motor, -MAX_SPEED);
            wb_robot_step(TIME_STEP);
          }
          break;
        }
      }

      // Para após dançar
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break;
    }

    // Anda aleatoriamente no começo
    if (!colidiu && timer-- == 0) {
      v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      wb_motor_set_velocity(left_motor, v_left);
      wb_motor_set_velocity(right_motor, v_right);
      timer = 100 + rand() % 100;
    }
  }

  wb_robot_cleanup();
  return 0;
}
