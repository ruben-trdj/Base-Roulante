/* CODEVSI Base Roulante - Groupe 4
* 
* Programme Arduino permettant de piloter les deux moteurs d'une base roulante afin de réaliser une trjectoire programmée via l'application associée
* 
* Pour le faire fonctionner: 
*   - Lancer la simulation
*   - Copiez-collez dans le moniteur la liste de vitesses présente dans le fichier texte issu de l'application
*   - Appuyez sur "Envoyé"
*   - Les deux moteurs tournent aux vitesses désirées afin de réaliser la trajectoire demandé
*
*/


int IN_G = 11;   //broches hacheur Enable et IN
int IN_D = 6;
int EN_G = 8;
int EN_D = 7;

int CODEURD = 2; //broches du codeur pour PID
int CODEURG = 3;
int pid = 1;

int TRIG = 13;   //broches capteur à Ultrasons
int ECHO = 4;

int FUMEE = A0;  //broche analog du détecteur de fumée

// DECLARATION DES VARIABLES 

char chiffre ;      // va prendre chaque caractère 1 à 1 du moniteur 
String nombre = ""; // va stocker chaque valeur 1 à 1 du moniteur pid;


int compteur_test_distance = 0;


// DECLARATION DES VARIABLES PID

int vitesse_reelleG, vitesse_reelleD;
int erreurG = 0, erreurD = 0;
int somme_erreurG = 0, somme_erreurD = 0;
int delta_erreurG = 0, delta_erreurD = 0;
int erreur_precedenteG = 0, erreur_precedenteD = 0;
int nouvelle_consigneG, nouvelle_consigneD;
int vitesseCG, vitesseCD;
int correctionG, correctionD;
int PWMG, PWMD;
int nombrevirgules ;

// CONSTANTES PID

float Kp = 0.4;
float Ki = 0.01;
float Kd = 0.001;

// FONCTION CAPTEUR DE CALCUL DE LA DISTANCE

int lecture_distance() //Fonction sans entrée qui renvoie la distance en cm calculée par le capteur IR.
	{ 
      
    int cm, lecture_echo; 
  
    digitalWrite(TRIG, HIGH); //Envoi du ping
    delayMicroseconds(0.1);  
    digitalWrite(TRIG, LOW);
  
    lecture_echo = pulseIn(ECHO,HIGH);
    cm = lecture_echo /58;    //Calcul distance en cm
    return(cm);
	}


void setup() 
	{  
// DECLARATION DES PINS DIGITAL
  
    pinMode(CODEURD,INPUT);
    pinMode(CODEURG, INPUT);
  
    pinMode(ECHO, INPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(FUMEE, INPUT);
  
    Serial.begin(9600);
  
    digitalWrite(EN_D,HIGH); //on enclenche les moteurs
    digitalWrite(EN_G,HIGH);  }




void loop()
	{  
// TEST CAPTEUR IR DE DISTANCE

    if (compteur_test_distance == 100)
    	{
  		if  (lecture_distance() < 20) //si le mur est à moins de 20cm
  			{
    		digitalWrite(EN_D,LOW);
    		digitalWrite(EN_G,LOW);
    		Serial.println("DISTANCE TROP COURTE");
    		compteur_test_distance = 0;
          	delay(3000);
  			}
  		}
        
        
  	else 
    	{
// COMMUNICATION INTERFACE - PILOTAGE MOTEUR

        if (analogRead(FUMEE) > 2) 
            {
            Serial.println("TROP DE FUMEE");
            }  
        if (Serial.available()) // tant qu'il reste des caractères dans le moniteur
            {
            chiffre = Serial.read();
          	//Serial.println(chiffre);
            if (chiffre == ',') // dans ce cas, nombre est "complet" et on peut exploiter sa valeur 
                {
                nombrevirgules = nombrevirgules + 1;
                if (nombrevirgules % 2 == 0)  
                    {            		
                  if (pid == 1){
                  	vitesse_reelleG = 9.95*((60*1000*10)/pulseIn(CODEURG,HIGH))/2; //Calcul de la vitesse
                    if (nombre.toInt() < 127) {vitesse_reelleG = - vitesse_reelleG;}
                    vitesseCG = map(nombre.toInt(), 0,255,-140,140);

                    erreurD = vitesseCG - vitesse_reelleG;                         //Calcul des erreurs
                    somme_erreurG = (somme_erreurG + erreurG)*0.01;
                    delta_erreurG = (erreurG - erreur_precedenteG)/0.01;
                    erreur_precedenteG = erreurD;

                    correctionG = Kp*erreurG + Ki*somme_erreurG + Kd*delta_erreurG;//Calcul nouvelle consigne corrigée
                    if(nombre.toInt() > 127){PWMG = vitesseCG + correctionG;}
                    if(nombre.toInt() < 127){PWMG = vitesseCG - correctionG;}

                    if(PWMG > 139){PWMG = 140;}
                    if(PWMG < -139){PWMG = -140;}
                    PWMG = map(PWMG, -140,140,0,255);
                    //Serial.println("test");
                  }
                  else{
                    PWMG = nombre.toInt();
                  }

                    //Affichage vitesse et erreur
                    //Serial.println("Nombre:");
                    //Serial.println(nombre.toInt());
                    //Serial.println("Vitesse:");
                    //Serial.println(map(vitesse_reelleG,-140,140,0,255));
                    //Serial.println("Consigne:");
                    //Serial.println(PWM);

                    
                    analogWrite(IN_G,PWMG);  // on donne la consigne au moteur

    				// on réinitialise la variable nombre
                    
                }else 
                {
                  if (pid == 1){


                    vitesse_reelleD = 9.95*((60*1000*10)/pulseIn(CODEURD,HIGH))/2; //Calcul de la vitesse
                    if (nombre.toInt() < 127) {vitesse_reelleD = - vitesse_reelleD;}
                    vitesseCD = map(nombre.toInt(), 0,255,-140,140);


                    erreurD = vitesseCD - vitesse_reelleD;                         //Calcul des erreurs
                    somme_erreurD = (somme_erreurD + erreurD)*0.01;
                    delta_erreurD = (erreurD - erreur_precedenteD)/0.01;
                    erreur_precedenteD = erreurD;


                    correctionD = Kp*erreurD + Ki*somme_erreurD + Kd*delta_erreurD;//Calcul nouvelle consigne
                    if(nombre.toInt() > 127){PWMD = vitesseCD + correctionD;}
                    if(nombre.toInt() < 127){PWMD = vitesseCD - correctionD;}

                    if(PWMD > 139){PWMD = 140;}
                    if(PWMD < -139){PWMD = -140;}
                    PWMD = map(PWMD, -140,140,0,255);
                  }
                    //Affichage vitesse et erreur
                    //Serial.println("Nombre:");
                    //Serial.println(nombre.toInt());
                    //Serial.println("Vitesse:");
                    //Serial.println(map(vitesse_reelleD,-140,140,0,255));
                    //Serial.println("Consigne:");
                    //Serial.println(PWM);
                  else{
                    PWMD = nombre.toInt();
                  }
                  
                    analogWrite(IN_D,PWMD);  // on donne la consigne au moteur

                              // on donne la consigne au moteur
                          // on réinitialise la variable nombre                    	  
                    }
              		nombre = ""; 

		
		} else 
                    {
                    if (chiffre != ' ' and chiffre != '[' and chiffre != ']') // afin d'éviter de prendre en compte les espaces dans la variable nombre
                        {  
                        nombre = nombre + chiffre;
                        }
                    }
            }
      	}
    compteur_test_distance = compteur_test_distance + 1;  //on incrémente le compteur pour le capteur de distance
	}
