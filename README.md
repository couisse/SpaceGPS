# SpaceGPS
 Un GPS dans l'espace

## Le programme
Ce programme a été conçu pour, à partir de la description d'un système solaire en deux dimensions et de la donnée de deux planètes (départ et arrivée), générer une trajectoire interplanétaire entre ces deux dernières, puis de la simuler par intégration d'Euler.
Ce programme utilise [SFML](https://www.sfml-dev.org/)

## Installation
Pour les utilisateurs de Windows, un exécutable est disponible [ici](https://github.com/couisse/SpaceGPS/releases/tag/win_alpha)
Sinon, il convient d'installer SFML (instructions d'installation [ici](https://www.sfml-dev.org/tutorials/2.6/) ) puis de compiler le projet sous C++11.

Pour le tester, vous pouver utiliser la commande:
`./spacegps.exe solar_system.csv Mercure auto 0 Terre 18000 time 9.81 1 out.csv`

## Utilisation
Le programme s'utilise à la ligne de commande sous deux formes (sgps est le nom d'appel du programme. En utilisant l'executable Windows, cela devient pour cmd `.\spacegps.exe`):
`sgps <commandsfilename>`
`sgps <systemfilename> <startname> <startheight(auto/double)> <starttime(double)> <targetname> <targetheight(auto/double)>
 <scoring(balance/time/deltav)>  <shipacceleration(double)> <shoulddisplay(1/0)> <outputfile>
`

Dans le premier cas, `commandsfilename` est le nom d'un fichier situé dans le répertoire `in`, comportant une suite de commandes sous l'un des format ci-dessus. Le prefixe d'appel est cependant omis.
Ex:
`.\spacegps.exe file.txt`, avec pour fichier `file .txt`:
```
solar_system.csv Terre 1000 0 Mars 1000 balance 9.81 1 out.csv
solar_system.csv Mercure auto 0 Terre auto time 9.81 1 out.csv
```
produira le calcul d'une trajectoire de la Terre à Mars, puis de Jupiter à Mars.


Dans le second cas, les arguments ont pour sens suivants:
+ `<systemfilename>` Le nom d'un fichier situé dans le répertoire `systems`. Ce fichier doit être un fichier csv comportant une ligne par planete avec pour colonnes:
    Astre /	Masse (kg)	/ Rayon (m)	/ a (m)	/ e	 / w (rad) / t0 (s) avec
    - Le nom de l'astre en question
    - La masse de l'astre, en kg
    - Le rayon de l'astre, en m. Un rayon approximatif est suffisant. Pour plus de précision, il est préférable d'inclure dans le rayon l'athmosphère dense.
    - Le demi grand axe de l'orbite de la planete autour de l'étoile
    - L'exentricité de l'orbite
    - L'angle périgée, en rad. La droite de référence pour l'angle n'a pas d'importance
    - Le temps POSIX d'un passage au périgé par l'astre (peut être négatif comme positif)

    La première ligne du fichier est spéciale, puisqu'elle représente l'étoile en elle même. Les valeurs a, e, w et t0 sont donc indéfinies et peuvent être remplies avec `-`. Le système solaire est fournie par défaut et peut servir d'exemple.
    **Attention** le comportement du programme face à un système stellaire non réaliste (eg. planètes trop proches, de gravité trop importante...) est indéfini. Il a été conçu à partir de la supposition que l'étoile est l'astre dominant gravitationnellement le système.

+ `<startname>` Le nom de la planète de départ
+ `<startheight(auto/double)>` L'altitude de départ du vaisseau en km, relativement au sol. Pour le simulateur, le vaisseau sera initialement placé en orbite circulaire à (startheight + rayon). Cela ne compte donc pas la phase de décollage, non prise en charge par le programme Si l'argument est `auto`, l'altitude choisie sera deux fois le rayon de l'astre (et donc l'orbite aura pour rayon trois fois celui de l'astre). A noter que ce choix n'est pas du tout optimal
+ `<starttime(double)>` Le temps POSIX de départ minimal du vaisseau. Noter que cela n'est pas le temps de départ réel (celui-ci est calculé par le GPS pour être optimal), mais c'est un minorant de ce temps de départ
+ `<targetname>` Le nom de  la planète d'arrivée
+ `<targetheight(auto/double)>` L'altitude cible en km pour l'arrivée. Le simulateur essayera de se placer en orbite circulaire à cette altitude en fin de voyage. `auto`a un comportement identique à celui de `<startheight>`. A noter que le programme à pour le moment beaucoup de difficultés à gérer cette étape
+ `<scoring(balance/time/deltav)>` La méthode de notation du GPS. `time` essaye de minimiser le temps de voyage, `deltav` le deltav consommé et `balance` le produit des deux
+ `<shipacceleration(double)>` L'accélération, en m/s², dont est capable le vaisseau
+ `<shoulddisplay(1/0)>` Indique si le programme doit invoquer une fenêtre SFML pour afficher la trajectoire. Dans le cas d'un fichier de commande, l'affichage d'une trajectoire bloque le processus. Pour passer à la commande suivante, il faut fermer la fenêtre
+ `<outputfile>` Le nom d'un fichier. Le programme écrira le résultat de l'exécution à la fin du fichier. Ce résultat est en fait (pour le moment) un profil des performances du programme et non l'enregistrement de la trajectoire (cette fonctionnalité est à venir).

## Résultat

Comme évoqué ci-dessus, le programme ne fourni pas pour le moment d'enregistrement direct de la trajectoire. La seule manière d'en observer les résultats est donc de spécifier `<shoulddisplay>` à 1. Les ellipses noires sont les orbites planétaires, les orbites rouges sont les orbites de transfert, et la courbe simulé est aussi affichée. `Espace` permet de faire avancer le temps, (pour l'instant il n'y a pas de marqueur sur la trajectoire simulée, seules les orbites bougent), `R` de retourner à t = 0, `ZQSD` de se déplacer. Le zoom est supporté, de même que le redimmensionnement de la fenêtre.