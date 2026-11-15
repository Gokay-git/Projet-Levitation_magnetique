"""
Simulation de la maquette sur PC : compile le vrai code du firmware
(Firmware/LM_V2/Core, fichiers LM*) avec un modele de la maquette, puis lance
les scenarios de test.

  python simuler.py              scenarios S1 a S4 (maintien, echelon, identification, defaut)
  python simuler.py courant      reponse de la boucle de courant seule
  python simuler.py ideal        S1 a S4 avec des courants parfaits (sans L293D)
  python simuler.py sansbiais    S1 a S4 sans courant de biais
  python simuler.py journal      journal serie complet -> journal_simulation.txt (pour Outils/lm_outil.py)

Necessite un compilateur C (gcc ou clang) dans le PATH.
"""
import argparse, glob, os, shutil, subprocess, sys, tempfile

ICI = os.path.dirname(os.path.abspath(__file__))
CORE = os.path.normpath(os.path.join(ICI, "..", "Firmware", "LM_V2", "Core"))
MODES = {"": "", "courant": "c", "ideal": "i", "sansbiais": "n", "journal": "L"}


def compilateur():
    for cc in ("gcc", "cc", "clang"):
        if shutil.which(cc):
            return cc
    sys.exit("Aucun compilateur C trouve (gcc, cc ou clang).\n"
             "Windows : installer MSYS2 puis 'pacman -S mingw-w64-ucrt-x86_64-gcc', ou utiliser WSL.")


def main():
    ap = argparse.ArgumentParser(description="Simulation de la maquette avec le vrai code du firmware")
    ap.add_argument("mode", nargs="?", default="", choices=list(MODES), help="scenario a lancer")
    a = ap.parse_args()

    build = tempfile.mkdtemp(prefix="lm_sim_")
    try:
        # main.h de remplacement dans le meme dossier que LM.h : c'est lui qui est inclus
        for f in glob.glob(os.path.join(CORE, "Inc", "LM*.h")) + glob.glob(os.path.join(CORE, "Src", "LM*.c")):
            shutil.copy(f, build)
        shutil.copy(os.path.join(ICI, "stub", "main.h"), build)
        shutil.copy(os.path.join(ICI, "sim.c"), build)
        exe = os.path.join(build, "sim.exe" if os.name == "nt" else "sim")
        sources = [os.path.join(build, f) for f in os.listdir(build) if f.endswith(".c")]
        cmd = [compilateur(), "-O2", "-std=gnu11", "-o", exe] + sources + ["-lm"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit("Echec de la compilation :\n" + r.stderr)

        if a.mode == "journal":
            sortie = os.path.join(ICI, "journal_simulation.txt")
            with open(sortie, "w", newline="") as f:
                subprocess.run([exe, MODES[a.mode]], stdout=f, check=True)
            print("journal ecrit :", sortie)
            print("analyse : python ../Outils/lm_outil.py journal_simulation.txt --R 11")
        else:
            subprocess.run([exe] + ([MODES[a.mode]] if MODES[a.mode] else []), check=True)
    finally:
        shutil.rmtree(build, ignore_errors=True)


if __name__ == "__main__":
    main()
