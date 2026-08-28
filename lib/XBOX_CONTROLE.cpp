#include <windows.h>   // Biblioteca do Windows
#include <Xinput.h>    // Biblioteca para ler controle Xbox
#include <iostream>    // Para usar cout
#include <cmath>       // Para usar abs()

//esta parte é somente um teste para o controle, a lógica dele
/*procurei por estas duas bibliotecas windows.h e Xinput.h, salvam a vida na hora dos testes, pelo oq eu estava olhando
era recomendado nos utilizarmos um controles bluetooth ao invés do USB do xbox360, muito mais simples de programar, no entanto
isto aq é meio q uma gambiarra pra conseguir testar a lógica usando meu próprio controle.
*/
#pragma comment(lib, "Xinput9_1_0.lib")
//isto aqui linka a biblioteca necessária para usar o XInput blz? 

/*a partir daqui pode repetir a logica, por exemplo se a gente precisar usar o bluetooth
a gente so pega essa parte do código e joga a desenvolve a conexão dos analogicos do bluetooth
com isto
*/
#define ZONAMORTA 2500 


int funcaonormalizar(SHORT valorBruto)
{
    // Se estiver dentro da zona morta, considera como zero!!!!!!!!!!!!!!!!!!!!!
    if (abs(valorBruto) < ZONAMORTA)
     return 0;

    // Converte para float para fazer conta mais precisa
    float valor = (float)valorBruto / 32767.00f;

    // Multiplica para virar escala de -1000 a 1000
    return (int)(valor * 1000.00f);
}
/*essa função de normalizar da pra literal copiar eu acho, oq vai mudar é a de baixo, q eu meio q peguei 
os analogicos de mão beijada pela propria biblioteca Xinput 
*/
int main()
{
    // Estrutura que guarda o estado do controle
    XINPUT_STATE estado; 
    ZeroMemory(&estado, sizeof(XINPUT_STATE)); //define para zero o incial 
        
        printf("MEXA OS ANALOGICOS DO CONROLE!!!\n");

    while (true)
    {
        // Verifica se o controle está conectado
        if (XInputGetState(0, &estado) == ERROR_SUCCESS)
        {
            // Leitura dos dois sticks (eixos X e Y)
            SHORT lx = estado.Gamepad.sThumbLX;
            SHORT ly = estado.Gamepad.sThumbLY;
            SHORT rx = estado.Gamepad.sThumbRX;
            SHORT ry = estado.Gamepad.sThumbRY;

            // Normaliza os valores
            int nlx = funcaonormalizar(lx);
            int nly = funcaonormalizar(ly);
            int nrx = funcaonormalizar(rx);
            int nry = funcaonormalizar(ry);

            // Mostra na tela (atualiza na mesma linha)
            
            int repetir;
            if (abs(nlx) > 0 || abs(nly) > 0 || abs(nrx) > 0 || abs(nry) > 0)
                repetir = 1;
            else
                repetir = 0;
            
            while (repetir == 1)
            {
            printf("ANALOGICO ESQUERDO EIXO X: %d\n", nlx);
            printf("ANALOGICO ESQUERDO EIXO Y: %d\n", nly);
            printf("ANALOGICO DIREITO EIXO X: %d\n", nrx);
            printf("ANALOGICO DIREITO EIXO Y: %d\n", nry);
            }
        }
        else
            printf("CONTROLE NAO CONECTADO...\n");
    
        Sleep(50); // atualiza a cada 50ms
    }
    return 0;
}
