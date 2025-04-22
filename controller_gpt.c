enum Estado {
  BUSCANDO_PAREDE,
  SEGUINDO_PAREDE,
  FAZENDO_CURVA
};

enum Estado estado = BUSCANDO_PAREDE;

int contador_voltas = 0;
double distancia_alvo = 30.0; // quanto mais alto, mais longe da parede

while (wb_robot_step(TIME_STEP) != -1) {
  // Atualiza sensores
  for (i = 0; i < QtddSensoresProx; i++) {
    LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
  }

  switch (estado) {
    case BUSCANDO_PAREDE:
      // Anda até detectar parede na frente
      wb_motor_set_velocity(MotorEsquerdo, 1.0);
      wb_motor_set_velocity(MotorDireito, 1.0);

      if (LeituraSensorProx[0] > distancia_alvo || LeituraSensorProx[7] > distancia_alvo) {
        estado = FAZENDO_CURVA;
      }
      break;

    case FAZENDO_CURVA:
      // Gira 90° para a direita
      wb_motor_set_velocity(MotorEsquerdo, 3.0);
      wb_motor_set_velocity(MotorDireito, -3.0);

      static int tempo_giro = 0;
      tempo_giro += TIME_STEP;

      if (tempo_giro > 500) { // ajuste fino do tempo de giro
        tempo_giro = 0;
        contador_voltas++;

        if (contador_voltas % 4 == 0) {
          distancia_alvo -= 10; // aproxima um pouco a espiral
          if (distancia_alvo < 40) distancia_alvo = 80; // reset para reiniciar espiral
        }

        estado = SEGUINDO_PAREDE;
      }
      break;

    case SEGUINDO_PAREDE:
      // Segue a parede pela direita
      if (LeituraSensorProx[1] < distancia_alvo) {
        // Tá longe da parede, vira pra direita
        wb_motor_set_velocity(MotorEsquerdo, 2.5);
        wb_motor_set_velocity(MotorDireito, 1.0);
      } else if (LeituraSensorProx[1] > distancia_alvo + 30) {
        // Tá grudado, vira pra esquerda
        wb_motor_set_velocity(MotorEsquerdo, 1.0);
        wb_motor_set_velocity(MotorDireito, 1.0);
      } else {
        // Distância boa, segue reto
        wb_motor_set_velocity(MotorEsquerdo, 3.0);
        wb_motor_set_velocity(MotorDireito, 3.0);
      }

      if (LeituraSensorProx[0] > distancia_alvo || LeituraSensorProx[7] > distancia_alvo) {
        estado = FAZENDO_CURVA;
      }
      break;
  }

  // debug
  printf("Estado: %d | ps0: %.2f ps1: %.2f ps7: %.2f\n", estado, LeituraSensorProx[0], LeituraSensorProx[1], LeituraSensorProx[7]);
}
