# Firmware LM_V2 — NUCLEO-L476RG

Projet STM32CubeIDE complet (CubeMX `LM_V2.ioc`, HAL FW_L4 V1.18.1).

## Ouvrir et compiler

STM32CubeIDE : **File → Open Projects from File System** → dossier `Firmware/LM_V2` → **Finish**,
puis **Project → Clean** et **Ctrl+B**. Le mode d'emploi détaillé (commandes, procédure de
réglage et d'identification) est dans `LM_V2/LISEZMOI.txt`.

## Fonctionnement

TIM1 génère la PWM centrée à 5 kHz et déclenche l'ADC1 à chaque période. Le DMA range les
8 mesures (4 courants, 3 capteurs Hall, tension de bus), puis la boucle complète s'exécute dans
l'interruption de fin de conversion : mesures → sécurités → correcteur de position →
correcteurs de courant → ponts L293D → log.

## Architecture du code (`LM_V2/Core`)

| Fichier | Rôle |
|---|---|
| `LM_config.h` | Tous les réglages : grandeurs à mesurer (R, L, ω₀, β) et choix de conception |
| `LM.c` | Séquencement de la boucle à 5 kHz, commandes, modes, défauts |
| `LM_measure.c` | ADC → courants (A), champ (mT), tension de bus (V), position X/Y |
| `LM_control.c` | PID de position par placement de pôles, pondération de consigne, PI de courant, courant de biais |
| `LM_bridge.c` | Commande des L293D (sens sur IN, PWM sur EN) |
| `LM_safety.c` | Surintensité, tension de bus, objet perdu |
| `LM_ident.c` | Identification en boucle fermée (balayage sinus, détection synchrone, moindres carrés) |
| `LM_log.c` | Log série 921600 bauds : données à 500 Hz, lignes `#CFG`, `#CAP`, `#BODE`, `#ID` |

Les fichiers suivent le gabarit STM32 (en-têtes Doxygen, sections *Includes / Private /
Exported*). `main.c` et les autres fichiers CubeMX ne contiennent du code utilisateur que dans
les sections `USER CODE` : le projet peut être régénéré depuis `LM_V2.ioc` sans perte.

## Réglage

Les valeurs de `LM_config.h` marquées « a mesurer » sont provisoires. Elles se mesurent sur la
maquette avec l'outil PC (dossier `Outils/`) : essai échelon pour L, balayage pour ω₀ et β.
