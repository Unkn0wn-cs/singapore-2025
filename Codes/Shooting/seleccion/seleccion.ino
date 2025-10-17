#include <Pixy2.h>
Pixy2 pixy;

const int NUM_FRANJAS = 3;
const int ANCHO_IMAGEN = 316; // Pixy2 horizontal resolution
int pesos[NUM_FRANJAS] = {0};

int classifyLane(float x, float y, bool right) {
    // Define the two diagonal boundaries
    float mA, mB, bA, bB;
    
    if (right){
      mA = (24.0 - 205.0) / (128.0 - 45.0);  // ≈ -2.346
      bA = 205 - mA * 45;            // ≈ 310.57

      mB = (180.0 - 24.0) / (237.0 - 188.0); // ≈ 2.636
      bB = 24 - mB * 188; 
    } else {
      mA = (61.0 - 182.0) / (144.0 - 6.0);  // ≈ -2.346
      bA = 182 - mA * 6;            // ≈ 310.57

      mB = (186.0 - 8.0) / (279.0 - 161.0); // ≈ 2.636
      bB = 8 - mB * 161;            // ≈ -500.2
    }


    // Evaluate position relative to lines
    float yA = mA * x + bA;
    float yB = mB * x + bB;

    Serial.print("antes de la formula ");
    Serial.println(yA);

    if (y < yA) {
        return 0; // Lane 1
    } else if (y >= yA && y < yB) {
        return 2; // Lane 2
    } else {
        return 1; // Lane 3
    }
}

void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.setLamp(0, 0);
}

void loop() {
  pixy.ccc.getBlocks();

  // Reset pesos
  for (int i = 0; i < NUM_FRANJAS; i++) {
    pesos[i] = 0;
  }

  if (pixy.ccc.numBlocks > 0) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1){
        Block block = pixy.ccc.blocks[i];

        // Clasifica en franja según posición X
        int franja = classifyLane(block.m_x, block.m_y, false);
        franja = constrain(franja, 0, NUM_FRANJAS - 1);

        // Calcula tamaño como área
        int tamano = block.m_width * block.m_height;

        // Suma al peso de la franja
        pesos[franja] += tamano;
      }
    }

    // Encuentra la franja con mayor peso
    int mejorFranja = 0;
    for (int i = 1; i < NUM_FRANJAS; i++) {
      if (pesos[i] > pesos[mejorFranja]) {
        mejorFranja = i;
      }
    }

    // Imprime decisión
    Serial.print("Franja más pesada: ");
    Serial.println(mejorFranja);
  }
  delay(500);
}