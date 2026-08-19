void ESC_BLDC(int16_t velocidade)
{ 
  if (velocidade < 0)
    velocidade = 0;

  if (velocidade > 1023)
    velocidade = 1023;

  int16_t vel_correspondente = map(velocidade, 0, 1023, 0, 180);
  ESC.write(vel_correspondente);
}
