# Introduction

Les dynamixels sont des servomoteurs ayant un angle de fonctionnement de 300° répartis sur des valeurs allant de 0 à 1023.

![alt text](electronics/actuators/Dynamixels_SRC/AngleFonctionnement.png )

Ils ont aussi la particularité de pouvoir être utilisés en tant que moteur DC. Ils ont un très bon couple moteur ce qui les rend efficaces pour un grand nombre d’applications. Dans nos robots, nous les avons utilisés pour leurs actions à réaliser pendant la compétition, à savoir :
*	Trier des balles de différentes couleurs
*	Déployer un bras servant à pousser un objet roulant
*	Déployer une plateforme sur plusieurs étages afin de libérer des blocs cubiques
*	Retenir ou non les blocs de chaque étage
*	Actionner un engrenage servant à déplacer des blocs cubiques

Le câblage se fait à l’aide de petits connecteurs 3pins. 

 ![alt text](electronics/actuators/Dynamixels_SRC/PinsConnections.png )

Les dynamixels possèdent 2 emplacement pour ces connecteurs car il est possible de connecter plusieurs servomoteurs en série et de les contrôler avec un adressage. Ces adresses sont représentées par des ID uniques pour chaque dynamixel qu’il est possible de vérifier à l’aide du Dynamixel Wizard du logiciel Roboplus et d’une interface USB2Dynamixel. 
Grâce à ce logiciel, il est également possible de configurer les dynamixels (registres) dont, entre autre, le baudrate du servomoteur ainsi que d’accéder à toute une série d’informations temps réel comme la vitesse, la position, etc.

# Spécifications techniques

*	Weight : 53.5g (AX-12/AX-12+), 54.6g (AX-12A)
*	Dimension : 32mm * 50mm * 40mm
*	Resolution : 0.29°
*	Gear Reduction Ratio :  254 : 1
*	Stall Torque : 1.5N.m (at 12.0V, 1.5A)
*	No load speed : 59rpm (at 12V)
*	Running Degree :  0° ~ 300° or Endless Turn
*	Running Temperature : -5℃ ~ +70℃
*	Voltage : 9  ~ 12V (Recommended Voltage 11.1V)
*	Command Signal : Digital Packet
*	Protocol Type : Half duplex Asynchronous Serial Communication (8bit,1stop,No Parity)
* Link (Physical) : TTL Level Multi Drop (daisy chain type Connector)
*	ID : 254 ID (0~253)
*	Communication Speed : 7343bps ~ 1 Mbps
*	Feedback : Position, Temperature, Load, Input Voltage, etc.
*	Material : Engineering Plastic


# Dynamixel Wizard

*	Télécharger le logiciel RoboPlus sur le site www.robotis.com en allant dans Support>Download>Software>Roboplus
 
 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Roboplus.png )
 
*	Lancer le logiciel et cliquer sur Dynamixel Wizard

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_DynamixelWizard.png )

*	Insérer l’interface USB2Dynamixel dans un port USB de l’ordinateur 
*	Brancher le connecteur 3 pins dans l’interface USB2Dynamixel (coté TTL) et dans le dynamixel à vérifier ou configurer
*	Sélectionner « TTL » à l’aide du switch sur l’interface USB2Dynamixel
*	Alimenter le dynamixel séparément dans sa plage de tension de fonctionnement (9-12V, recommandé 11,1V) à l’aide du 2eme port du dynamixel

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_USB2Dynamixel.png )

*	Sélectionner le port de votre ordinateur auquel est branché l’USB2Dynamixel

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Port.png )

*	Cliquer ensuite sur « Ouvrir le port »

![alt text](electronics/actuators/Dynamixels_SRC/DW_OuvrirLePort.png ) 

* Faire une recherche basique du dynamixel afin de trouver son ID


*	Il est également possible de faire des recherches plus avancées à d’autre baudrate dans le cas ou la recherche basique n’est pas concluante
*	Une fois le dynamixel trouvé, le sélectionner à gauche de la fenêtre. 

*	Le détail des informations du dynamixel s’affiche

*	Dans ce détail, il est possible de modifier la configuration du dynamixel comme par exemple son ID, sa vitesse de communication (baudrate), sa vitesse de fonctionnement, etc, ou encore de le contrôler en temps réel.


# Exemples d’utilisation

Pour pouvoir utiliser les dynamixels, il est nécessaire au préalable de télécharger la librairie « AX_12A_servo_library » se trouvant sur github
Il faudra ensuite l’extraire et la déplacer dans Documents/Arduino/libraries.

Cette librairie possède 4 exemples d’utilisation du dynamixels : 
-	Blink : exemple de contrôle de la led rouge se trouvant à l’arrière du dynamixel
-	Move : exemple de contrôle du dynamixel en mode servomoteur en lui attribuant des positions
-	EndlessTurn : utilisation du dynamixel en mode moteur DC (ou wheel mode)
-	ReadRegister : sert à afficher le contenu des registres du dynamixel

Ces exemples se trouvent dans Fichier>Exemples>AX-12A.

Après avoir sélectionner un des exemples, il faut inclure la librairie AX-12A sans déclarer un chemin spécifique car la librairie se trouve dans le dossier « libraries » du dossier « Arduino ».

 ![alt text](electronics/actuators/Dynamixels_SRC/Exemples_Include.png )

Ensuite il suffit de configurer 3 lignes dans le code qui répondront aux besoins et à la configuration du dynamixel branché :

#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            (1u)

DirectionPin sert à indiquer le sens de communication du dynamixel, 10u étant l’écriture dans les registres il n’est pas nécessaire de modifier sa valeur.

BaudRate définit la vitesse de communication utilisée (et configurée préalablement à l’aide du Dynamixel Wizard).

ID représente l’ID du dynamixel préalablement vérifiée ou configurée.

# Références

-	e-manual détaillé de l’ensemble des gammes de dynamixel le AX-12A qui nous interesse ici : http://support.robotis.com/en/ 

-	vidéo youtube reprenant différentes étapes de l’utilasation du Dynamixel Wizard : https://www.youtube.com/watch?v=YJ9b68hx5Qc&version=3&hl=ko_KR

-	site internet du fabricant : http://en.robotis.com

-	récupération de la librairie AX-12A : https://github.com/ThingType/AX-12A-servo-library

