# Outil PC : lm_outil.py

Lit le log série envoyé par la Nucleo (921600 bauds) ou un fichier de log enregistré, puis :
- enregistre le journal (`journal.txt`) et les données (`donnees.csv`) ;
- calcule **L** d'une bobine à partir d'un essai échelon (`#CAP`) ;
- calcule **ω₀ et β** d'un axe à partir d'un balayage d'identification (`#BODE` / `#ID`)
  et affiche les lignes à recopier dans `LM_config.h` ;
- trace les courbes (PNG).

Les résultats sont rangés dans un dossier `mesures_AAAAMMJJ_HHMMSS`.

## Installation

Python 3, plus :
- `pyserial` pour lire la carte en direct ;
- `matplotlib` pour les courbes (facultatif : sans lui, l'outil calcule tout mais ne trace rien).

```
pip install pyserial matplotlib
```

Avec Anaconda, `matplotlib` est déjà installé.

## Utilisation

Depuis le dossier `Outils` :

```
python lm_outil.py journal_exemple_simulation.txt --R 11   # essai sans matériel
python lm_outil.py --ports                                  # liste des ports série
python lm_outil.py COM5 --R 10.4                            # carte en direct (Ctrl+C pour arrêter)
```

`--R` est la résistance de la bobine mesurée à l'ohmmètre, en ohms (elle sert au calcul de L).

Dans VS Code, ouvrir le dossier `Outils` : les 3 commandes ci-dessus sont prêtes dans
**Run and Debug** (`.vscode/launch.json`).
