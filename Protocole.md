# Asservissement : protocole de communication

## Feedback continue

L'asservissement envoie à intervalle régulier des infos sur son état.
Le format du statut est le suivant :

`"#<positionX>;<positionY>;<angle>;<statutConsigne>;<vitesseG>;<vitesseD>\r\n"`

| Paramètre     | Type    | Description                        | Unité   |
|---------------|---------|------------------------------------|---------|
| positionX     | int32_t | Coordonnée en X du robot           | mm      |
| positionY     | int32_t | Coordonnée en Y du robot           | mm      |
| angle         | double  | Cap du robot                       | radians |
| commandStatus | int32_t | Statut du gestionnaire de commande | N/A     |
| vitesseG      | int32_t | Consigne de vitesse moteur gauche  | N/A     |
| vitesseD      | int32_t | Consigne de vitesse moteur droit   | N/A     |

Les valeurs de `vitesseG` et `vitesseD` sont les consignes directement
envoyées à la carte de contrôle moteurs. La valeur dépend donc de la
carte utilisée. Pour la MD22, par exemple, les valeurs vont de -128 à 127.

Les valeurs que peut prendre `commandStatus` sont les suivantes :

| Nom              | Valeur | Description
|------------------|--------|--------------------------------------------
| `STATUS_IDLE`    |  0     | Aucune consigne en cours, en attente.
| `STATUS_RUNNING` |  1     | Une consigne est en cours d'exécution.
| `STATUS_HALTED`  |  2     | Un arrêt d'urgence a été demandé.
| `STATUS_BLOCKED` |  3     | Une consigne est en cours, mais le robot semble coincé / ne bouge plus.

> **ATTENTION !**
> Lors de l'envoi d'une commande, le statut renvoyé peut ne pas passer de
> `STATUS_IDLE` à `STATUS_RUNNING` immédiatement. Le changement de statut
> n'est pas synchronisé avec l'ajout de la commande, mais avec son traitement
> par la boucle d'asservissement. L'IA qui interagit avec l'asservissement
> peut également lire le statut avec un peu de retard. Pour éviter les
> problèmes, l'IA devrait attendre d'avoir reçu **deux fois** le statut
> `STATUS_IDLE` après avoir envoyé une commande pour être sûr que la
> commande demandée est réellement terminée !

## Commandes

Chaque commande est composée d'un code de 1 à 3 caractères, suivie
de 0 à 3 paramètres.

Le format est le suivant :

`<code>[<param1>[#<param2>]...]`

Il n'y a donc pas de séparateur entre le code et le premier paramètre,
et tous les paramètres sont séparés par des `#`.

Si une commande prend des paramètres, c'est précisé. Sinon, elle n'en
prend pas.

### Commandes basiques

`h`: Arrêt d'urgence. Le robot s'arrête et n'accepte plus aucune
    consigne. Seul une commande `r` peut faire repartir le robot.

`r`: Reset de l'arrêt d'urgence

`v`: Avance

* Paramètre :

| Type    | Description          | Unité |
|---------|----------------------|-------|
| int64_t | Distance à parcourir | mm    |

`t`: Tourne

* Paramètre :

| Type    | Description                              | Unité  |
|---------|------------------------------------------|--------|
| int64_t | Angle de rotation (sens trigonométrique) | degrés |

### Commandes GOTO

Toutes les commandes de ce type prennent les mêmes paramètres.

| Type    | Description                | Unité |
|---------|----------------------------|-------|
| int64_t | Coordonnés en X d'un point | mm    |
| int64_t | Coordonnés en Y d'un point | mm    |

`go`: Goto. Va au point demandé, en marche avant.

`ge`: Goto enchainé. Comme un Goto, mais si la consigne suivante est
        aussi un Goto ou un Goto enchainé, on s'autorise passer à la
        commande suivante si on est pas loin du point d'arrivé.

`gf`: Faire face à un point. Tourne le robot pour se placer face au
        point demandé, mais ne fait pas avancer ou reculer le robot.

`gb`: Goto Back. Comme un Goto, mais en marche arrière.

### Commandes de feedback

`p`: Récupérer la position. Renvoie la position sur la série avec le
    format suivant :

`"x<positionX>y<positionY>a<angle>\r\n"`

| Paramètre | Type    | Description              | Unité   |
|-----------|---------|--------------------------|---------|
| positionX | int32_t | Coordonnée en X du robot | mm      |
| positionY | int32_t | Coordonnée en Y du robot | mm      |
| angle     | double  | Cap du robot             | radians |

### Commandes de controle de l'odométrie

`Osa` : Définir l'angle pour l'odométrie. L'odométrie pensera que son
    angle courant est à la nouvelle valeur. Le robot ne bouge pas.

* Paramètre:

| Type   | Description | Unité   |
|--------|-------------|---------|
| double | Angle       | radians |

`Osx` : Définir la position en X pour l'odométrie. L'odométrie pensera
    que sa position en X est à la nouvelle valeur. Le robot ne bouge pas.

* Paramètre:

| Type    | Description    | Unité |
|---------|----------------|-------|
| int64_t | Coordonné en X | mm    |

`Osy` : Définir la position en Y pour l'odométrie. L'odométrie pensera
    que sa position en Y est à la nouvelle valeur. Le robot ne bouge pas.

* Paramètre:

| Type    | Description    | Unité |
|---------|----------------|-------|
| int64_t | Coordonné en Y | mm    |

### Commandes de controle des régulateurs

`Rle` : Vitesse basse sur les régulateurs

`Rld` : Vitesse normale sur les régulateurs

`Rae` : Activation régu angle

`Rad` : désactivation régu angle

`Rar` : Reset régu angle

`Rde` : Activation régu distance

`Rdd` : désactivation régu distance

`Rdr` : Reset régu distance

### Commandes de configuration

`CR`: Réinitialiser l'asserv

`CD`: Dump de la config

`CG`: Lire (Get) la valeur d'un paramètre

`CS`: Définir (Set) la valeur d'un paramètre

`CL`: Charger (Load) la config

`CW`: Ecrire (Write) la config
