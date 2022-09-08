# TMC-2130-Arduino-Hardware-and-Software

[![arduino-library-badge](https://www.ardu-badge.com/badge/StepperDriver.svg?)](https://www.ardu-badge.com/StepperDriver)
[![Actions Status](https://github.com/laurb9/StepperDriver/workflows/PlatformIO/badge.svg)](https://github.com/laurb9/StepperDriver/actions)
[![Actions Status](https://github.com/laurb9/StepperDriver/workflows/Arduino/badge.svg)](https://github.com/laurb9/StepperDriver/actions)

Hardware
=============
Le TMC2130 est un Drivers pour moteur pas a pas pour la gestion des moteurs pas a pas .Il fonctionne presque comme le A4988.Principalement utiliser pour les imprimantes et les machines CCN.
Il est facile a prendres en charges lorsque le connection Hardware est bien faite.

Il possede 2 modes de fonctionnement principaux qui sont le Mode Standalone et le mode SPI.
Certains models sont fabriqué par defauts avec le mode SPI d'autes par contres avec le mode SPI

versions: V1.0, V1.1, V1.2

   - V1.0 default is set up for SPI, 
   - V1.1 default is set up for Standalone mode. 
   - V1.2 is prepared with some of the pins going up. Default is set up for SPI mode

Hardware currently supported: 
   Picture 
[alt tag](https://user-images.githubusercontent.com/111455408/189115534-207be335-79f7-4b4c-8ff1-3e5a14f02948.PNG)

Comment reconnaitre une carte fabriqué par defaut avec le mode Standalone??

[alt tag](https://user-images.githubusercontent.com/111455408/189115516-87866e53-8dd2-49a2-89aa-52026fc1c7f9.jpg)

Pour le modele V1.1 il n’y as pas de connection pour CFG4 et CFG5.
Puis la presence de la resistance R5

Mais ce mode n'est pas vraiment efficacce et les parmetres sont delicats a mettre en ouevre ;Je vous conseil donc d'utiliser le mode SPI pour vos 
application qui necessite de la precision et un control absolu
Dans mon cas j’ai  utilisé le model V1.1  dont dispose Makerlab
En effet par defaut il est en Mode Standalone

Pour passer au mode SPI,il Faut retirer  la resistance R5 de 0 ohm avec un fer chaud puis etablir une conection pour les broches des Registres CFG4 et CFG5 comme si dessous.


Lorseque la carte est par defaut en mode SPI il n’y aucune modification a faire

Parametres
==========
Puis plusieurs parametres pour lesquels les moteurs se comportent differement 
-Stealh chop
-Cool step
-Spread cycle

Pour fixer des parametres par defauts pour la consomation de courant  il faut adjuser la tension grace au potentiometre
Par exemple 
0.3A => 0.42V on the adjusting screw
0.6A => 0.85V on the adjusting screw
Avec la formule ci dessous
Retenez que plus la consomation est grande plus la puissance du moteur est grande.Il y’a cependant une limite.



Motors
======

- 4-wire bipolar stepper motor or 


SPI
===
Une liaison SPI (Serial Peripheral Interface) est une liaison série synchrone créée par Motorola et qui fonctionne en full duplex 
(les deux circuits peuvent communiquer en même temps sur le même bus) . Comme pour la liaison I2C, la communication est réalisée selon un schéma maître-esclaves, 
où le maître s'occupe totalement de la communication. Plusieurs esclaves peuvent être reliés au même bus et la sélection du destinataire se fait par une ligne appelée 
Slave Select (SS).

Le bus SPI utilise quatre signaux logiques :

 -   SCLK : Serial Clock, Horloge (généré par le maître).

 -   MOSI : Master Output, Slave Input (généré par le maître).

 -   MISO : Master Input, Slave Output (généré par l'esclave).

 -   SS : Slave Select, Actif à l'état bas (généré par le maître).

La communication sur le bus est orchestrée de la manière suivante :

    Le maître sélectionne l'esclave avec lequel il souhaite communiquer en mettant un niveau bas sur la ligne SS correspondante.

    Le maître génère le signal d'horloge en fonction des capacités de l'esclave

    A chaque coup d'horloge, le maître et l'esclave s'échangent un bit sur les lignes MOSI et MISO selon le principe ci-dessous.

L'Arduino Uno possède une liaison SPI (SCLK : broche numérique N°13, MISO : broche numérique N°12, MOSI : broche numérique N°11 et
 SS : broche numérique N°10 et autres si nous avons plusieurs composants esclaves) 


Pinout
======
 #### Pin Functions  
 
    POWER SUPPLY 
   
    GND  ----->  GROUND

    VM    ----->  MOTOR SUPPLY VOLTAGE (5.5V - 46V)
  
    VIO    ---->  LOGIC SUPPLU VOLTAGE  (3.3V -5V)


 #### MOTOR  OUTPUTS

  M1A  ---->  MOTOR Bobine 1 (Black)
  
  M1B   ---->  MOTOR Bobine 1 (Green)
  
  M2A    ---->   MOTOR COIL 2 (Red)
  
  M2B     ---->   MOTOR COIL 2 (blue)

###### CONTROL INPUTS 
  
STEP   ---->  Step Signal  Inputs 

 DIR     ---->   Dir  signal Inputs
 
 EN      ---->   Enable Motor Outputs (GND=on, VIO=off, OPEN=Auto-Power-Down)

#### TMC2130 SPI MODE (SPI jumper Ouvert)

SDO/CFG0  ---->   MISO - Serial Data Output    (pin 11 )

SDI/CFG1    ---->   MOSI - Serial Data Input        (pin 12)

SCK/CFG2  ---->   SCLK - Serial Clock Input,      (pin 13

CS/CFG3    ---->    SS - Chip Select Input         (pin 10)


SOFTWARE
====
Je tiens a preciser que j'utilise un arduino Uno et les pin pour le SPI Varie according to your Arduino board
 -D’abord  pour tester si la connection SPI est fonctionnelle
Telechercher la bibliotheque TMCStepper by teemuatlut  en clickant ici link puis la bibliotheque Accel Stepper disponible dans le gestionnaire de bibliotheque Arduino IDE.


```C++
/**
 * Author Cedric Francois
 * Initializes the library and Show on the serial monito the DRVstatus sor SPI coonection.
*/

#define EN_PIN    5  
#define DIR_PIN   6  
#define STEP_PIN  7  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13

bool dir = true;

#include <TMC2130Stepper.h>
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Start...");
  driver.begin();       
  driver.rms_current(800);  
  
  digitalWrite(EN_PIN, LOW);

  Serial.print("DRV_STATUS=0b");
  Serial.println(driver.DRV_STATUS(), BIN);
}

void loop() {

}
```

Copier et coller sur arduino IDE.
Open the serial monitor 
si the DRV_STATUS=0b0 alors le cablage SPI est mal fait.
Veuillez verifier votre cablage


- Pour faire tourner le moteur de facon simple,continu et avec une vitesse par defaut Eun this code
La fonction "driver.shaft_dir" de la bibliotheque TMC2130Stepper.h permet d'inverser la direction de rotation du moteur ;
 

```C++
#define EN_PIN    5  
#define DIR_PIN   6 
#define STEP_PIN  7  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13

bool dir = true;

#include <TMC2130Stepper.h>
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);

void setup() {
	Serial.begin(9600);
	while(!Serial);
	Serial.println("Start...");
	driver.begin(); 	
	driver.rms_current(600); 	// Courrant du moteur
	driver.stealthChop(1); 	
	
	digitalWrite(EN_PIN, LOW);
}

void loop() {
	digitalWrite(STEP_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(STEP_PIN, LOW);
	delayMicroseconds(10);
	uint32_t ms = millis();
	static uint32_t last_time = 0;
	if ((ms - last_time) > 2000) {
		if (dir) {
			Serial.println("Dir -> 0");
			driver.shaft_dir(0);
		} else {
			Serial.println("Dir -> 1");
			driver.shaft_dir(1);
		}
		dir = !dir;
		last_time = ms;
	}
}
```
- Pour controller en temps reel les parametres du moteur,Ce code vous permettra d'envoyer des donneés par la liaso UART du moniteur Serie meme lorseque le moteur est en plein fonctionnement grace a certainne fonction de la bibliotheque TMC2130Stepper
par exemple
La commande run 0 permet d'arreter le moteur puis Run 1 le remetre en marche
La commande speed permet d'envoyer la valeur de  vitesse 10-speed-4649
shaft_dir inverse le sens de rotation

ect....
Puis tour terminer,Afin de controler
Lien Externe
============
- A <a href="https://www.pololu.com/category/120/stepper-motor-drivers">stepper motor driver</a>, for example DRV8834, DRV8825, DRV8824, A4988.
- A <a href="http://www.circuitspecialists.com/stepper-motor">Stepper Motor</a>.

