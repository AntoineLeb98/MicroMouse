#include <IRremote.hpp>
#define IR_receiver 12

struct Commande {
  int direction;   // 0=UP, 1=RIGHT, 2=DOWN, 3=LEFT
  int nbCases;
};

Commande fileCommandes[10];
int nbCommandesRecues = 0;
int directionEnAttente = -1; // -1 = pas encore de direction reçue

void afficherFileCommandes() {
  Serial.println("--- File de commandes ---");
  for (int i = 0; i < nbCommandesRecues; i++) {
    Serial.print(i);
    Serial.print(" : direction=");
    Serial.print(fileCommandes[i].direction);
    Serial.print(" nbCases=");
    Serial.println(fileCommandes[i].nbCases);
  }
  Serial.println("-------------------------");
}

int codeVersDirection(int code) {
  if (code == 0x46) return 0; // UP
  if (code == 0x43) return 1; // RIGHT
  if (code == 0x15) return 2; // DOWN
  if (code == 0x44) return 3; // LEFT
  return -1; // pas une touche direction
}

int codeVersChiffre(int code) {
  if (code == 0x16) return 1;
  if (code == 0x19) return 2;
  if (code == 0xD)  return 3;
  if (code == 0xC)  return 4;
  if (code == 0x18) return 5;
  if (code == 0x5E) return 6;
  if (code == 0x8)  return 7;
  if (code == 0x1C) return 8;
  if (code == 0x5A) return 9;
  return -1; // pas un chiffre
}

void receptionCmdIR() {
  if (IrReceiver.decode()) {
    int code = IrReceiver.decodedIRData.command;

    if (code == 0x40) {
      // OK -> termine la séquence, lance le déplacement (pour plus tard, étape 4)
      // rien à faire ici pour l'instant
      afficherFileCommandes();
    }
    else if (code == 0x52) {
      // recommencer la séquence -> vide la file
      nbCommandesRecues = 0;
      directionEnAttente = -1;
    }
    else if (code == 0x42 || code == 0x4A) {
      // arrêt d'urgence -> pour plus tard (étape 2, moteurs)
    }
    else if (directionEnAttente == -1) {
      // On attend une direction
      int dir = codeVersDirection(code);
      if (dir != -1) {
        directionEnAttente = dir;
      }
    }
    else {
      // On attend un chiffre
      int chiffre = codeVersChiffre(code);
      if (chiffre != -1 && nbCommandesRecues < 10) {
        fileCommandes[nbCommandesRecues].direction = directionEnAttente;
        fileCommandes[nbCommandesRecues].nbCases = chiffre;
        nbCommandesRecues++;
        directionEnAttente = -1;
      }
    }

    IrReceiver.resume();
  }
}

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_receiver);
  Serial.println("Pret a recevoir - appuie sur les boutons un par un");
}

void loop() {
  receptionCmdIR();
}