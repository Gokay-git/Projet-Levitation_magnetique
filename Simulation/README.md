# Simulation de la maquette sur PC

`simuler.py` compile le **vrai code du firmware** (`Firmware/LM_V2/Core`, fichiers `LM*`) pour le PC
et le fait tourner sur un modèle de la maquette, période par période à 5 kHz. Il permet de tester
une modification du firmware avant de la flasher sur la carte.

## Ce qui est modélisé

| Élément | Modèle |
|---|---|
| Objet | 2 axes instables : p'' = ω₀²·p + β·u + perturbation |
| Bobines | 4 circuits R-L (R = 11 Ω, L = 25 mH) |
| Ponts L293D | PWM sur EN (impulsion centrée), chute de tension du pont, roue libre par les diodes, conduction discontinue autour de 0 A |
| Mesures | ADC 12 bits, offsets et bruit sur les courants et les capteurs Hall |

La maquette simulée (ω₀ = 25 rad/s, β = 70 000) est volontairement différente du réglage provisoire
de `LM_config.h` (30 rad/s, 90 000) : la simulation vérifie aussi que l'identification retrouve
les bonnes valeurs.

Fichiers : `sim.c` (modèle et scénarios), `stub/main.h` (remplace le `main.h` STM32 : le minimum
de la HAL pour compiler sur PC), `simuler.py` (compilation et lancement).

## Utilisation

Il faut un compilateur C dans le PATH : `gcc` (Linux, WSL, ou MSYS2 sous Windows) ou `clang`.

```
python simuler.py              # scénarios S1 à S4
python simuler.py courant      # réponse de la boucle de courant seule
python simuler.py ideal        # S1 à S4 avec des courants parfaits (sans L293D)
python simuler.py sansbiais    # S1 à S4 sans courant de biais
python simuler.py journal      # journal série complet pour l'outil PC
```

Le mode `journal` écrit `journal_simulation.txt`, à analyser avec
`python ../Outils/lm_outil.py journal_simulation.txt --R 11`.

## Résultats avec le firmware V2

| Scénario | Résultat |
|---|---|
| S1 : maintien puis perturbation constante sur X | PID : erreur finale −0,002 mT (le PD seul garde 0,685 mT d'écart), aucun défaut, courant max 0,23 A |
| S2 : échelon de consigne +1 mT | b = 1 : dépassement 53 %, 154 ms ; **b = 0,5 : dépassement 4 %, 63 ms** |
| S3 : identification sur X | ω₀ = 25,11 rad/s et β = 70 592 trouvés pour 25 et 70 000 réels ; après application : dépassement 4 %, 77 ms |
| S4 : réglage incohérent (β < 0) | lévitation refusée, défaut 5 comme prévu |
| Boucle de courant, sinus de 0,1 A à 25 Hz | gain 0,82 autour de 0 A, **0,99 autour de 0,2 A** (courant qui ne passe plus par 0, comme avec le courant de biais) |
