// não podee  ser uma função que guarda posição, pois deve ter loop.
//Vel dow e vel rise são velocidades de decida e subida do drone e com a vel liquida final é só somar a altura encotrada pelo barometro, e mpu
unsigned long tempo_anterior = 0;
// Velocidades
float vel_x = 2.0; //2m/s
float vel_y = 2.0;// 2m/s
float vel_rise = 2.0;//2m/s
float vel_down = 1.5;//1.5m/s
// Posição do drone final
float pos_x = 0.0;
float pos_y = 0.0;
float pos_z = 0.0;//Altura
void loop()
{
    // Tempo atual
    unsigned long tempo_atual = micros();

    // Comandos recebidos do controle TODOS em +1 ,-1 ou 0
    int Comando_x=0;
    int Comando_y=0;
    int Comando_z =0;

    // Tempo decorrido
    unsigned long Timer_= tempo_atual -tempo_anterior;
    // Converter para segundos
    float Timer_ = Timer_/1000000.0;
//Eixo X
    if (Comando_x== 1){pos_x +=vel_x*Timer_;}
    if (Comando_x ==-1){pos_x-=vel_x*Timer_;}
//Eixo Y
    if (Comando_y == 1){pos_y+= vel_y*Timer_;}
    if (Comando_y ==-1){pos_y -=vel_y*Timer_;}
// Eixo Z 
    if (Comando_z== 1){pos_z +=vel_rise*Timer_;}
    if (Comando_z == -1){pos_z -= vel_down*Timer_;}


    // Atualizar o tempo anterior
    tempo_anterior=tempo_atual;
}
