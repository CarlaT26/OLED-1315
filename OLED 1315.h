#include "mbed.h"
#include "Adafruit_SSD1306.h"  // Asegúrate de tener instalada esta librería

// Definición de tamaño del display (modifica según tu pantalla)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Configuración del bus I2C (modifica los pines según tu placa)
I2C i2c(D14, D15); // SCL = D14, SDA = D15

// Configuración del display OLED usando I2C
Adafruit_SSD1306_I2c display(i2c, NC, 0x7A, SCREEN_WIDTH, SCREEN_HEIGHT);  // Sin pin de reset


int main() {
    // Inicializa la pantalla
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x7A)) }
    {
        printf("No se pudo inicializar la pantalla OLED.\n");
        while (true);
    }
    
    // Limpia la pantalla
    display.clearDisplay();
    
    // Escribe un texto en la pantalla
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("Pantalla OLED OK!");
    
    // Actualiza la pantalla con el contenido
    display.display();

    // Mantén el programa corriendo
    while (true) {
        // Puedes agregar más lógica aquí si lo deseas
    }
}
