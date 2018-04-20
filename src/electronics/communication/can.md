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

On se rend bien compte l'importance de l'électronique et au vu du nombre croissants de ces derniers, le câblage fut vite un vrai problème. Ainsi, BOSCH, a mis développé le CANBus pour répondre 



# Liens utiles

