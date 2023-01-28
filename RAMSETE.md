Trucs à vérifier lorsque l'on fait un controlleur Ramsete, en plus de ce que la [documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html), qu'il faut respecter en tout point, recommande:

## Odométrie

- L'angle donnée doit être dans l'intervalle [-180, +180]. Ce n'est pas le cas de la plupart des méthodes du Navx. `.getYaw()` est une exception et donne le bon intervale. Si l'on veut utiliser `.getFusedHeading()`, il faut ajuster.
- Évidemment, vérifier que l'odométrie retourne les bons angles dans le bon sens (positif pour des angles dans la direction inverse d'une montre), ainsi que les x et les y.
- À l'initialisation, le navx donne un angle de 0 degrès, ce qui fausse l'initialisation de l'odométrie qui s'attends à la place à la valeur réelle de l'angle. Donc, avant d'utiliser le navx, il faut attendre qu'il s'initialise. Par exemple:

```java
    private final AHRS navx = newNavx();

    private static AHRS newNavx() {
        var navx = new AHRS(SPI.Port.kMXP);

        if (Robot.isReal()) {
            while (Math.abs(navx.getYaw()) < 0.0001) {
                try {
                    Thread.sleep(20);
                }
                catch(InterruptedException ie) {
                    // Do nothing.
                }
            }
        }
        return navx;
    }
```

- La position des moteurs doit être en mètres, et une augmentation positive doit correspondre au robot qui avance. C'est le cas pour l'odométrie, mais pas mal tout le reste aussi. Donc il est recommandé d'avoir une méthode qui le fait de manière centrale, et d'utiliser cette méthode partout. Ex avec un moteur droit qui a besoin d'être inversé:

```
    private double positionMoteurDroitMetres() {
        if (Robot.isReal()) {
            return transmissionActive.distanceMetresPourCoches(-capteursMoteurDroit.getIntegratedSensorPosition());
        }
        else if (sim != null) {
            return sim.positionMoteurDroitMetres();
        }
        return 0;
    }
```

Note: inverser les contrôles des moteurs avec `moteurDroit.setInverted(true)` n'a aucune influence sur les encodeurs embarqués.

## Caractérisation

- Il est conseillé de faire la caractérisation plusieurs fois, par exemnple 3 fois, au cas où l'une d'entre elle donne des données franchement différentes. On peut prendre les données de la caractérisation avec le meilleur Sim Velocity R2.
- Il y a un bug lorsque l'on charge un fichier de caractérisation dans Sysid2023: le coefficient Kp pour une boucle fermée sur la vitesse (Velocity) est mauvais tant qu'on ne change pas la liste déroulante "Gain preset" pour "WPILib (2020-)". Bien penser à changer la liste déroulante avant de noter le coéfficient.
- Utiliser la caractérisation en rotation pour obtenir une mesure fiable de la largeur de la base ("Track width"). En effet, la mesure à la main n'est pas fiable, et est [une cause de soucis avec le controlleur](https://www.chiefdelphi.com/t/ramsete-controller-final-heading-error/374742/10?u=jul).

## Code du contrôleur

- S'assurer de bien configurer les TalonFX avec [un échantillonage rapide](https://www.chiefdelphi.com/t/limiting-voltage-pathweaver-and-ramesetecommand/405377/12?u=jul). En effet, le contrôleur PID est [très sensible au retard des capteurs](https://www.chiefdelphi.com/t/limiting-voltage-pathweaver-and-ramesetecommand/405377/11?u=jul).
- Déverminage: [calculer et afficher les contributions de chaque gain](https://www.chiefdelphi.com/t/limiting-voltage-pathweaver-and-ramesetecommand/405377/28?u=jul) pour voir lequel pourrait être problématique.
