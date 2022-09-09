# TMC-2130-Arduino-Hardware-and-Software

Le TMC2130 est un Drivers pour moteur pas a pas pour la gestion des moteurs pas a pas .Il fonctionne presque comme le A4988.Principalement utiliser pour les imprimantes et les machines CNC.
Il est facile a prendres en charges lorsque le connection Hardware est bien faite.

Il possede 2 modes de fonctionnement principaux qui sont le Mode Standalone et le mode SPI.
Certains models sont fabriqué par defaut avec le mode SPI d'autes par contres avec le mode SPI

versions: V1.0, V1.1, V1.2

   - V1.0 default is set up for SPI, 
   - V1.1 default is set up for Standalone mode. 
   - V1.2 is prepared with some of the pins going up. Default is set up for SPI mode

Hardware currently supported: 
   Picture 
![alt tag](https://user-images.githubusercontent.com/111455408/189115534-207be335-79f7-4b4c-8ff1-3e5a14f02948.PNG)


Comment reconnaitre une carte fabriqué par defaut avec le mode Standalone??

![alt tag](https://user-images.githubusercontent.com/111455408/189115516-87866e53-8dd2-49a2-89aa-52026fc1c7f9.jpg)

![alt tag](https://user-images.githubusercontent.com/111455408/189115529-d56c8d7a-2f6e-4246-ac13-0faff4bea9e2.jpg)


Pour le modele V1.1 il n’y as pas de connection pour CFG4 et CFG5.
Puis la presence de la resistance R5

Mais ce mode n'est pas vraiment efficacce et les parmetres sont delicats a mettre en ouevre ;Je vous conseil donc d'utiliser le mode SPI pour vos 
application qui necessite de la precision et un control absolu
Dans mon cas j’ai  utilisé le model V1.1  dont dispose Makerlab
En effet par defaut il est en Mode Standalone

Pour passer au mode SPI,il Faut retirer  la resistance R5 de 0 ohm avec un fer chaud puis etablir une conection pour les broches des Registres CFG4 et CFG5 comme si dessous.

![alt tag](https://user-images.githubusercontent.com/111455408/189160040-91af5ae8-5779-48a6-acf4-d558fd49ba38.jpg)


Lorseque la carte est par defaut en mode SPI il n’y aucune modification a faire

Parametres
==========

#### Mode d'operation

2-phase stepper motors up to 2.0A coil current (2.5A peak)

- **Step/Direction Driver Mode** using combinaison between TMC2130 and TMC429


- **Standalone Mode**

- **SPI Driver Mode**

 #### Control and precision

- **stallGuard2** for high precision sensorless motor load detection

- **Cool Step** current control for energy savings up to 75% 

- **dcStep**  Dependent speed control 

 #### Motion

- **spreadCycle**  highly dynamic motor contro

- **stealthChop** for extremely quiet operation and smooth motion



Pour fixer des parametres par defauts pour la consomation de courant  il faut adjuser la tension grace au potentiometre
Par exemple 
0.3A => 0.42V on the adjusting screw
0.6A => 0.85V on the adjusting screw
Avec la formule ci dessous

![Formule et Parametres](https://github.com/teemuatlut/TMC2130Stepper/blob/master/README.md)


Retenez que plus la consomation est grande plus la puissance du moteur est grande.Il y’a cependant une limite.


Motors
======

4-wire bipolar stepper motor 


![alt tag](https://user-images.githubusercontent.com/111455408/189160349-22c79bc9-b64d-4aab-bc36-427224533852.PNG)

![alt tag](https://user-images.githubusercontent.com/111455408/189159308-c68a0001-e52f-4c9b-be46-5636d29a0059.jpg)

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

 - Le maître sélectionne l'esclave avec lequel il souhaite communiquer en mettant un niveau bas sur la ligne SS correspondante.

 -Le maître génère le signal d'horloge en fonction des capacités de l'esclave

 - A chaque coup d'horloge, le maître et l'esclave s'échangent un bit sur les lignes MOSI et MISO selon le principe ci-dessous.

   **_L'Arduino Uno possède une liaison SPI (SCLK : broche numérique N°13, MISO : broche numérique N°12, MOSI : broche numérique N°11 et
    SS : broche numérique N°10 et autres si nous avons plusieurs composants esclaves)_**


Pinout
======  
 
 ###### POWER SUPPLY 
   
    GND  ----->  GROUND

    VM    ----->  MOTOR SUPPLY VOLTAGE (5.5V - 46V)
  
    VIO    ---->  LOGIC SUPPLU VOLTAGE  (3.3V -5V)


 ###### MOTOR  OUTPUTS

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

![alt tag](https://user-images.githubusercontent.com/111455408/189376870-82b2cf56-af73-4a9e-a835-b4c68e4d5674.PNG)


![alt tag](https://user-images.githubusercontent.com/111455408/189161743-83649f21-36ce-4a46-a1dd-a20a02fd744b.jpg)



SOFTWARE
========
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

```C++
#define EN_PIN    5  
#define DIR_PIN   6  
#define STEP_PIN  7  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13

int speed = 50;
bool running = false;
float Rsense = 0.11;
float hold_x = 0.5;
boolean toggle1 = 0;

#include <TMC2130Stepper.h>
TMC2130Stepper myStepper = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    digitalWrite(STEP_PIN, HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(STEP_PIN, LOW);
    toggle1 = 1;
  }
}

void initTimer() {
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS11);// | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

void setTimer(int t) {
  cli();
  TCNT1 = 0;
  OCR1A = t;
  sei();
}

void serialTuple(String cmd, int arg) {
  Serial.print("Received command: ");
  Serial.print(cmd);
  Serial.print("(");
  Serial.print(arg);
  Serial.println(")");
}


void setup() {
  initTimer();
  Serial.begin(9600);
  myStepper.begin();
  myStepper.SilentStepStick2130(1000);
  myStepper.stealthChop(1);   // Enable extremely quiet stepping
  digitalWrite(EN_PIN, LOW);
  Serial.println("Setup ready");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil(' ');
    String strArg = Serial.readStringUntil('\n');

    int arg = strArg.toInt();

    if (cmd == "run") {
      serialTuple("run", arg);
      running = arg;
      arg ? digitalWrite(EN_PIN, LOW) : digitalWrite(EN_PIN, HIGH);
    }
    else if (cmd == "speed") {
      serialTuple("speed", arg);
      //speed = arg;
      setTimer(arg);
    }
    else if (cmd == "setCurrent") {
      serialTuple("setCurrent", arg);
      myStepper.setCurrent(arg, Rsense, hold_x);
    }
    else if (cmd == "Rsense") {
      Serial.print("Setting R sense value to: ");
      Serial.println(arg);
      Rsense = arg;
    }
    else if (cmd == "hold_multiplier") {
      Serial.print("Setting hold multiplier to: ");
      Serial.println(arg);
      hold_x = arg;
    }
    else if (cmd == "GCONF") {
      Serial.print("GCONF: 0b");
      Serial.println(myStepper.GCONF(), BIN);
    }
    else if (cmd == "external_ref") {
      serialTuple("external_ref", arg);
      myStepper.external_ref(arg);
    }
    else if (cmd == "internal_sense_R") {
      serialTuple("internal_sense_R", arg);
      myStepper.internal_sense_R(arg);
    }
    else if (cmd == "stealthChop") {
      serialTuple("stealthChop", arg);
      myStepper.stealthChop(arg);
    }
    else if (cmd == "commutation") {
      serialTuple("commutation", arg);
      myStepper.commutation(arg);
    }
    else if (cmd == "shaft_dir") {
      serialTuple("shaft_dir", arg);
      myStepper.shaft_dir(arg);
    }
    else if (cmd == "diag0_errors") {
      serialTuple("diag0_errors", arg);
      myStepper.diag0_errors(arg);
    }
    else if (cmd == "diag0_temp_prewarn") {
      serialTuple("diag0_temp_prewarn", arg);
      myStepper.diag0_temp_prewarn(arg);
    }
    else if (cmd == "diag0_stall") {
      serialTuple("diag0_stall", arg);
      myStepper.diag0_stall(arg);
    }
    else if (cmd == "diag1_stall") {
      serialTuple("diag1_stall", arg);
      myStepper.diag1_stall(arg);
    }
    else if (cmd == "diag1_index") {
      serialTuple("diag1_index", arg);
      myStepper.diag1_index(arg);
    }
    else if (cmd == "diag1_chopper_on") {
      serialTuple("diag1_chopper_on", arg);
      myStepper.diag1_chopper_on(arg);
    }
    else if (cmd == "diag1_steps_skipped") {
      serialTuple("diag1_steps_skipped", arg);
      myStepper.diag1_steps_skipped(arg);
    }
    else if (cmd == "diag0_active_high") {
      serialTuple("diag0_active_high", arg);
      myStepper.diag0_active_high(arg);
    }
    else if (cmd == "diag1_active_high") {
      serialTuple("diag1_active_high", arg);
      myStepper.diag1_active_high(arg);
    }
    else if (cmd == "small_hysteresis") {
      serialTuple("small_hysteresis", arg);
      myStepper.small_hysteresis(arg);
    }
    else if (cmd == "stop_enable") {
      serialTuple("stop_enable", arg);
      myStepper.stop_enable(arg);
    }
    else if (cmd == "direct_mode") {
      serialTuple("direct_mode", arg);
      myStepper.direct_mode(arg);
    }
    else if (cmd == "hold_current") {
      serialTuple("hold_current", arg);
      myStepper.hold_current(arg);
    }
    else if (cmd == "run_current") {
      serialTuple("run_current", arg);
      myStepper.run_current(arg);
    }
    else if (cmd == "hold_delay") {
      serialTuple("hold_delay", arg);
      myStepper.hold_delay(arg);
    }
    else if (cmd == "power_down_delay") {
      serialTuple("power_down_delay", arg);
      myStepper.power_down_delay(arg);
    }
    else if (cmd == "hold_delay1") {
      Serial.print("microstep_time: ");
      Serial.println(myStepper.microstep_time());
    }
    else if (cmd == "stealth_max_speed") {
      serialTuple("stealth_max_speed", arg);
      myStepper.stealth_max_speed(arg);
    }
    else if (cmd == "coolstep_min_speed") {
      serialTuple("coolstep_min_speed", arg);
      myStepper.coolstep_min_speed(arg);
    }
    else if (cmd == "mode_sw_speed") {
      serialTuple("mode_sw_speed", arg);
      myStepper.mode_sw_speed(arg);
    }
    else if (cmd == "coil_A_current") {
      serialTuple("coil_A_current", arg);
      myStepper.coil_A_current(arg);
    }
    else if (cmd == "coil_B_current") {
      serialTuple("coil_B_current", arg);
      myStepper.coil_B_current(arg);
    }
    else if (cmd == "DCstep_min_speed") {
      serialTuple("DCstep_min_speed", arg);
      myStepper.DCstep_min_speed(arg);
    }
    else if (cmd == "CHOPCONF") {
      Serial.print("CHOPCONF: 0b");
      Serial.println(myStepper.CHOPCONF(), BIN);
    }
    else if (cmd == "off_time") {
      serialTuple("off_time", arg);
      myStepper.off_time(arg);
    }
    else if (cmd == "hysteresis_start") {
      serialTuple("hysteresis_start", arg);
      myStepper.hysteresis_start(arg);
    }
    else if (cmd == "fast_decay_time") {
      serialTuple("fast_decay_time", arg);
      myStepper.fast_decay_time(arg);
    }
    else if (cmd == "hysteresis_low") {
      serialTuple("hysteresis_low", arg);
      myStepper.hysteresis_low(arg);
    }
/*    else if (cmd == "sine_offset") {
      serialTuple("sine_offset", arg);
      myStepper.sine_offset(arg);
    }*/
    else if (cmd == "disable_I_comparator") {
      serialTuple("disable_I_comparator", arg);
      myStepper.disable_I_comparator(arg);
    }
    else if (cmd == "random_off_time") {
      serialTuple("random_off_time", arg);
      myStepper.random_off_time(arg);
    }
    else if (cmd == "chopper_mode") {
      serialTuple("chopper_mode", arg);
      myStepper.chopper_mode(arg);
    }
    else if (cmd == "blank_time") {
      serialTuple("blank_time", arg);
      myStepper.blank_time(arg);
    }
    else if (cmd == "high_sense_R") {
      serialTuple("high_sense_R", arg);
      myStepper.high_sense_R(arg);
    }
    else if (cmd == "fullstep_threshold") {
      serialTuple("fullstep_threshold", arg);
      myStepper.fullstep_threshold(arg);
    }
    else if (cmd == "high_speed_mode") {
      serialTuple("high_speed_mode", arg);
      myStepper.high_speed_mode(arg);
    }
    else if (cmd == "sync_phases") {
      serialTuple("sync_phases", arg);
      myStepper.sync_phases(arg);
    }
    else if (cmd == "microsteps") {
      serialTuple("microsteps", arg);
      myStepper.microsteps(arg);
    }
    else if (cmd == "interpolate") {
      serialTuple("interpolate", arg);
      myStepper.interpolate(arg);
    }
    else if (cmd == "double_edge_step") {
      serialTuple("double_edge_step", arg);
      myStepper.double_edge_step(arg);
    }
    else if (cmd == "disable_short_protection") {
      serialTuple("disable_short_protection", arg);
      myStepper.disable_short_protection(arg);
    }
    else if (cmd == "sg_min") {
      serialTuple("sg_min", arg);
      myStepper.sg_min(arg);
    }
    else if (cmd == "sg_max") {
      serialTuple("sg_max", arg);
      myStepper.sg_max(arg);
    }
    else if (cmd == "sg_step_width") {
      serialTuple("sg_step_width", arg);
      myStepper.sg_step_width(arg);
    }
    else if (cmd == "sg_current_decrease") {
      serialTuple("sg_current_decrease", arg);
      myStepper.sg_current_decrease(arg);
    }
    else if (cmd == "smart_min_current") {
      serialTuple("smart_min_current", arg);
      myStepper.smart_min_current(arg);
    }
    else if (cmd == "sg_stall_value") {
      serialTuple("sg_stall_value", arg);
      myStepper.sg_stall_value(arg);
    }
    else if (cmd == "sg_filter") {
      serialTuple("sg_filter", arg);
      myStepper.sg_filter(arg);
    }
    else if (cmd == "stealth_amplitude") {
      serialTuple("stealth_amplitude", arg);
      myStepper.stealth_amplitude(arg);
    }
    else if (cmd == "stealth_gradient") {
      serialTuple("stealth_gradient", arg);
      myStepper.stealth_gradient(arg);
    }
    else if (cmd == "stealth_freq") {
      serialTuple("stealth_freq", arg);
      myStepper.stealth_freq(arg);
    }
    else if (cmd == "stealth_freq") {
      serialTuple("stealth_freq", arg);
      myStepper.stealth_freq(arg);
    }
    else if (cmd == "stealth_autoscale") {
      serialTuple("stealth_autoscale", arg);
      myStepper.stealth_autoscale(arg);
    }
    else if (cmd == "stealth_symmetric") {
      serialTuple("stealth_symmetric", arg);
      myStepper.stealth_symmetric(arg);
    }
    else if (cmd == "standstill_mode") {
      serialTuple("standstill_mode", arg);
      myStepper.standstill_mode(arg);
    }
    else if (cmd == "DRVSTATUS") {
      Serial.print("DRVSTATUS: 0b");
      Serial.println(myStepper.DRV_STATUS(), BIN);
    }
    else if (cmd == "PWM_SCALE") {
      Serial.print("PWM_SCALE: 0b");
      Serial.println(myStepper.PWM_SCALE(), BIN);
    }
    else if (cmd == "invert_encoder") {
      serialTuple("invert_encoder", arg);
      myStepper.invert_encoder(arg);
    }
    else if (cmd == "maxspeed") {
      serialTuple("maxspeed", arg);
      myStepper.maxspeed(arg);
    }
    else if (cmd == "interpolate") {
      serialTuple("interpolate", arg);
      myStepper.interpolate(arg);
    }
    else if (cmd == "LOST_STEPS") {
      Serial.print("LOST_STEPS: 0b");
      Serial.println(myStepper.LOST_STEPS(), BIN);
    }
    else {
      Serial.println("Invalid command!");
    }
  }
/*  
  if (running) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(speed);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(speed);
  }
*/
}
```

Puis tour terminer,Afin de controler la posotion l'utilisation de la bibliothe accel stepper est necessaire en combinaison avec TMC2130Stepper puis on utilise la fonction move(XXX) 
XXX :
compris un valeur entre 0 et 3200 puis 1/16
compris un valeur entre 0 et 1600 puis 1/8

 
```C++
#define EN_PIN    5  
#define DIR_PIN   6  
#define STEP_PIN  7  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13

constexpr uint32_t steps_per_mm = 80;

#include <TMC2130Stepper.h>
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setup() {
   // SPI.begin();
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Start...");
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    driver.begin();             // Initiate pins and registeries
    driver.rms_current(600); 
      driver.SilentStepStick2130(1000);
    driver.stealthChop(1);      // Enable extremely quiet stepping
    driver.stealth_autoscale(1);
    driver.microsteps(16);
    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
}

void loop() {
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
        delay(100);
        stepper.move(100*steps_per_mm); // Move 100mm
        stepper.enableOutputs();
    }
    stepper.run();
}
```

Lien Externe
============
-  <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiMk-_cyYX6AhX3iv0HHQ9LAfUQFnoECAMQAQ&url=https%3A%2F%2Fgithub.com%2Fteemuatlut%2FTMC2130Stepper&usg=AOvVaw2TLMbLapVpaJjcwKECfuLY
">Libray TMC2130Stepper by teemuatlut</a>.

-  <a href="https://reprap.org/wiki/TMC2130">TMC2130</a>.

-  <a href="https://www.omc-stepperonline.com/download/17HS24-2104S.pdf">17HS24-2104S MOTEUR</a>.


-  <a href="https://www.microcontrollertutorials.com/2021/07/tmc2130-stepper-motor-driver-working.html">TMC Hardware setup</a>.


- <a href="https://www.davidpilling.com/wiki/index.php/TMC2130">TMC AVEC GRBL </a>.

- <a href="https://pecquery.wixsite.com/arduino-passion/la-liaison-spi">SPI</a>.

- <a href="https://reprap.org/forum/read.php?2,864566">Standalone </a>.

- <a href="https://www.instructables.com/Fix-Cloned-Arduino-NANO-CNC-Shield/?fbclid=IwAR0JBdvT4k7ZmE-LR8plWQRc3csEvETMqG4wlkIfXm_4Cg-SzEGztKuENK8">Probleme de certains GRBL V4</a>.

