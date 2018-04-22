# Comprendre et implémenter le CANBus

## Contexte et problématique

De nombreuses nouvelles technologies ont été impléentées sur les robots lors de l'édition 2018 de l'eurobot. 

Ces dernières ont constitué une avancée non négligeable en terme de qualité, de rapidité et de simplicité. 
Cependant, des inconvénients inatendus sont apparus lors de l'implémentation et la convergence des technologies sur les robots.
Ces inconvénients sont des tous types mais ceux que l'on va retenir pour ce tutoriel sont d'ordre **électronique** et **physique**.

![alt text](./CAN_SRC/complex-wiring.png "Logo Title Text 1" )

### Problème électronique

Une fois installées, l'ensemble des cartes nécessitent de nombreuses connexions entres elles. La dernière image fait allusion au résultat final du câblage sur les robots. 

Electroniquement, cela induit une grande complexité lorsqu'une nouvelle connexion doit être faite. Cette complexité se fait également ressentir pour le débuggage car il est difficil de prendre des mesures et déterminer d'où vient l'erreur.

### Problème physique

Un autre problème qui est apparut et qui peut être moins intuitif ou du moins difficilement anticipable est l'espace pris par les câbles. Lors de la conception, **l'erreur** qu'on a fait, est de ne pas prendre en compte le câblage. Il serait opportun à l'avenir de déterminer des marges (coefficients) lors des calculs de dimensions et d'espace. Cette erreur a généré chez nous un nombre incalculable de câble ne suivant pas de chemin. 
L'autre problème physique avec un nombre important de connexions est la résistance mécanique de ces connexions. En effet, sur le petit robot, nous avons observé de nombreux "bugs" du à la déconnexion non voulue des câbles où au fait que certain fil de "désoudaient" du au trop grand nombre de câble présents et exercants une force dans les différents espaces restraints. 


## Le CANBus

### Contexte d'apparition

Les problèmes évoqués précédemment ont déjà été rencontrés il y a plusieurs années dans le milieu de l'automobile. En effet, au temps où l'électronique a fait son apparition dans les voitures, le nombre de composants intelligents n'ont cessé d'augmenter durant des années. Les composants électroniques étaient utilisés à tous les niveaux : 

* Températures moteurs, air...
* Régime moteur, vitesse engagées ...
* Gyroscope, accéléromètre...
* Contrôle sécurité, ABS, Airbag, portes ouvertes, ceintures...

### Caratéristiques et avantages

On se rend bien compte l'importance de l'électronique et au vu du nombre croissants de ces derniers, le câblage fut vite un vrai problème. Ainsi, BOSCH, a mis au point le CANBus avec de nombreuses caractéristiques avantageuses : 

* Grande quantité de données jusqu'a 1 Megabit/s (voir plus bas en fonction des distances)
* Temps réel (point fort par rapport au TCP/IP)
* Détection d'erreurs, rapide récupération et réparation (toujours temps réel)
* Grande stabilité et sécurtié
* Priorisation de transmission
* Basé sur un signal différentiel ce qui lui procure sa robustesse aux environnements sévères (Bruits électromagnétiques et tolérance aux pannes)
* Utilise un câble torsadé ce qui limite l'émission de bruit

Ci-dessous, les vitesses de transmissions possibles en fonction de la longueur du bus.

![alt text](./CAN_SRC/can_speed.PNG )


#### Topologie 

Enfin, le plus grand avantage du CANBus est sa topologie *une ligne* ce qui réduit drastiquement le nombre de câble. C'est d'ailleur la raison pour laquelle le CANBus est utilisé dans toutes les voitures modernes.

![alt text](./CAN_SRC/can_topology.png )

Ce qu'on observe sur la figure précédente, est le passage d'un système sans CANBus (à gauche) à un systèe avec CANBus (à droite). On remarque qu'une ligne traverse l'ensemble  du système à laquelle chacun des composants (noeud) va se connecter pour intérargir comme représenté à la figure suivante : 

![alt text](./CAN_SRC/can_busnode.jpg )

On peut faire l'analogie avec des personnes dans une pièces où tout le monde crie pour se faire entendre. Un système de priorité permet de départager les personnes voulant parler en même temps.

### Consitutation d'une trame

![alt text](./CAN_SRC/can_frame.jpg)

Une trame de donnée est constituée de plusieurs parties

* 1 bit dominant caractérisant le début d'une trame
* L'ID du message composé de 11 bit dans le CAN standart ou 29 bits dans le CAN étendu
* Un champ de commande de 6 bits qui détermine la longueur du champs de données
* Les données composée de 0 à 8 octets (64 bits)
* Un champs CRC pour la détection d'erreur
* Un champs d'acquittement
* Bit de fin de trame

### Priorité des transmissions

Lorsque plusieurs noeud veulent communiquer en même temps sur le Bus, c'est le champ d'arbitrage qui va déterminer la priorité. Elle sera donné au premier noeud présentant un bit dominant (0) alors que les autres noeuds présentent un bit récessif. 

Cela confère un grand avantage au canbus, il est possible de faire de la priorité de transmission en choisissant un bon ID. Par exemple, dans une voiture, le système de frein est bien plus important que le système de chauffage. L'ID du système de frein se constitué d'un nombre beaucoup plus bas (beaucoup de 0) que celui du système de chauffage.


## Implémentation

### Hardware

![alt text](./CAN_SRC/can_hardware.png)

La figure précedente présente l'implemntation hadware du CANBus. Elle est facilité par l'existence de modules tout fait développés par différentes entreprises. Un noeud est constitué d'un CAN transceiver (émetteur-récepteur), un CAN controlleur, un microcontrolleur et éventuellement de résistance de fin de lignes.

#### Transceiver

Microchip propose un grand nombre de solutions pour l'automobile. Parmi celles-ci un transceiver. 

Il s'agit du MCP2562. Anciennement MCP2551, il sert à convertir le signal TTL en signal différentiel exigé par le standart CAN.

#### Controller 

Microchip propose le controlleur CAN MCP2515 qui implémente toute les spécifications CAN 2.0. Il est capable d'envoyer et de recevoir les données et de communiquer avec en SPI avec un microcontrolleur.

#### Micro-controlleur
Il est peut être choisi selon les envies et spécifications du système du moment qu'il puissent communiquer en SPI.

Un ATMega328p très populaire peut être pris comme exemple.

#### Resistance

Comme dans toutes les liaisons, il existe de la réflexion qui peut gêner le système. Celle-ci peut être évitée par le placement de résistance, dimensionnée par BOSCH, de 120 Ohm en fin de ligne.

#### PCB

En cours de développpement, documentation bientôt disponible.

### Software (Arduino)

Il existe plusieurs librairies proposant l'utilisation du MCP2515 avec un microcontroleur.

Une libraire complète et mise à jour régulièrement est di 
<https://github.com/Seeed-Studio/CAN_BUS_Shield>



#### Filtres

Les filtres permettent de choisir quels messages vont être *écoutés* par le controlleur CAN et envoyés au microcontrolleur.

Le MCP2552 permet de définir 6 filtres. Exemple avec la définition de 2 filtres :

```javascript
CAN.init_Filt(0, 0, 0x04);                          
CAN.init_Filt(1, 0, 0x05);                          
```

Ce code induit que le controlleur n'écoutera que les messages avec l'ID 0x04 ou 0X05.

#### Masque
Les masques définissent les bits que l'on va regarder pour filtrer les ID entrant. Si le bit est à 1, l'ID doit respecter le filtre, sinon pas.

Exemple avec le filtre suivant :

```javascript
0100 1101
```

Si je définis mon masque à 0XFF, soit
```javascript
1111 1111
```

Cela veut dire que l'ID entrant doit être strictement pareil que le filtre car tous les bits du masques sont à 1. Donc seul un message avec l'ID

```javascript
0100 1101
```
sera accepté.

Si maintenant le masque est le suivant :

```javascript
1111 1110
```

Cela veut dire que le masque le filtre ne regarde pas le dernier bit d'un ID.

Les ID suivant seront donc accpetés :

```javascript
0100 1100
0100 1101
```

# Liens utiles

- Introduction au CANBus par Texas Instrument, <http://www.ti.com/lit/an/sloa101b/sloa101b.pdf> 
- Spécification CAN par BOSCH, <https://www.kvaser.com/software/7330130980914/V1/can2spec.pdf>
- Implémentation CANBus avec Arduino, <http://www.prometec.net/wp-content/uploads/2015/07/Controller-Area-Network-Prototyping-With-Arduino-Wilfried-Voss.pdf>
- CANBus sur Wikipédia (très bien documenté en Fr et En), https://en.wikipedia.org/wiki/CAN_bus

