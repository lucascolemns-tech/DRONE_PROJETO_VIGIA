#include <windows.h>
#include <Xinput.h>
#include <cstdio>  
#include <cmath>   
#include <iostream>

#pragma comment(lib, "Xinput9_1_0.lib") 

#define ZONAMORTA 2500 

int funcaonormalizar(SHORT valorBruto)
{
    if (abs(valorBruto) < ZONAMORTA)
        return 0;

    float valor = (float)valorBruto / 32767.00f;
    return (int)(valor * 1000.00f);
}

int main()
{
    XINPUT_STATE estado;
    ZeroMemory(&estado, sizeof(XINPUT_STATE));

    std::cout << "MEXA OS ANALOGICOS DO CONTROLE!!!\n";
    std::cout.flush();

    while (true)
    {
        if (XInputGetState(0, &estado) == ERROR_SUCCESS)
        {
            SHORT lx = estado.Gamepad.sThumbLX;
            SHORT ly = estado.Gamepad.sThumbLY;
            SHORT rx = estado.Gamepad.sThumbRX;
            SHORT ry = estado.Gamepad.sThumbRY;

            int nlx = funcaonormalizar(lx);
            int nly = funcaonormalizar(ly);
            int nrx = funcaonormalizar(rx);
            int nry = funcaonormalizar(ry);

            std::cout << nlx << "," << nly << "," << nrx << "," << nry << "\n";
            std::cout.flush();
        }
        else
        {
            std::cout << "CONTROLE NAO CONECTADO...\n";
            std::cout.flush();
        }

        Sleep(50);
    }
    return 0;
}