#!/usr/bin/env python3
"""
lm_outil.py - outil PC du projet LM (levitation magnetique, NUCLEO-L476RG)

Lit le log serie du firmware (921600 bauds), l'enregistre, et exploite
automatiquement les blocs speciaux :
  #CFG   reglage du correcteur de chaque axe (affiche en tableau)
  #CAP   essai echelon sur une bobine -> constante de temps tau, puis L = tau x R
  #BODE  balayage d'identification -> trace de Bode et droite de Re(1/G)
  #ID    resultat de l'identification : w0 et beta a recopier dans LM_config.h

Utilisation
  python lm_outil.py COM5                 lit la carte en direct (Ctrl+C pour arreter)
  python lm_outil.py COM5 --R 10.4        idem, avec la resistance mesuree a l'ohmmetre
  python lm_outil.py journal.txt          relit un journal deja enregistre
  python lm_outil.py --ports              liste les ports serie

Le port de la Nucleo s'appelle "STMicroelectronics STLink Virtual COM Port"
dans le Gestionnaire de peripheriques. Fermer PuTTY / Tera Term avant.
Chaque session cree un dossier mesures_AAAAMMJJ_HHMMSS avec :
  journal.txt (tout le flux), donnees.csv (500 Hz), cap_*.csv, bode_*.csv, *.png
Dependances : pyserial (lecture en direct), matplotlib (graphiques, facultatif).
"""
import argparse
import csv
import math
import os
import sys
import time

FS = 5000.0          # frequence de la boucle (LM_F_LOOP_HZ)
CAP_PRE = 64         # echantillons avant l'echelon (LM_CAP_PRE)
DATA_HEAD = ["n", "iX1_mA", "iX2_mA", "iY1_mA", "iY2_mA", "px_cmT", "py_cmT",
             "vbus_mV", "mode", "defaut"]
CFG_HEAD = ["axe", "kp A/mT", "ki A/(mT.s)", "kd mA.s/mT", "b", "tf ms", "wn rad/s",
            "Routh", "courant/wn", "ok"]
MODES = {0: "arret", 1: "tension", 2: "courant", 3: "levitation", 4: "defaut"}
DEFAUTS = {0: "aucun", 1: "surintensite", 2: "Vbus bas", 3: "Vbus haut",
           4: "objet perdu", 5: "reglage incoherent"}

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:   # matplotlib absent : on se passe des graphiques
    plt = None


# ------------------------------------------------------------------ analyses
def analyse_cap(rows, coil, mode, R_mesure, dossier, num):
    """rows : liste (k, consigne, courant_mA). Ajuste i(t) = i0 + (if - i0)(1 - exp(-t/tau))."""
    nom = os.path.join(dossier, "cap_%d_bobine%d.csv" % (num, coil))
    with open(nom, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "t_ms", "consigne_mV_ou_mA", "courant_mA"])
        for k, r, i in rows:
            w.writerow([k, "%.2f" % (k / FS * 1000), r, i])
    print("  -> %s" % nom)

    if mode != 1:
        print("  Essai en mode courant : c'est la reponse de la boucle de courant, pas de la bobine.")
        print("  Pour mesurer L, refaire l'essai en mode tension (lm.mode = 1).")
        return
    pre = [i for k, r, i in rows if k < CAP_PRE]
    post = [(k, i) for k, r, i in rows if k >= CAP_PRE]
    if len(pre) < 5 or len(post) < 50:
        print("  Capture trop courte pour l'analyse.")
        return
    i0 = sum(pre) / len(pre)
    n_fin = max(10, len(post) // 5)
    i_fin = sum(i for k, i in post[-n_fin:]) / n_fin
    di = i_fin - i0
    v_pas = rows[-1][1] / 1000.0   # V
    if abs(di) < 20:
        print("  Variation de courant trop faible (%.0f mA) : augmenter lm.step_value." % di)
        return
    # tau par regression de ln(1 - y) sur la zone 10 % - 90 %
    xs, ys = [], []
    for k, i in post:
        y = (i - i0) / di
        if 0.1 <= y <= 0.9:
            xs.append((k - CAP_PRE) / FS)
            ys.append(math.log(1.0 - y))
    if len(xs) < 4:
        print("  Montee trop rapide pour 5 kHz (tau < 1 ms ?) : tau non mesurable ici.")
        return
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    pente = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    tau = -1.0 / pente
    t_fin = (post[-1][0] - CAP_PRE) / FS
    R_app = v_pas / (di / 1000.0) if di else float("nan")
    print("  Echelon de %.2f V : courant %.0f -> %.0f mA" % (v_pas, i0, i_fin))
    print("  tau = %.2f ms   (fenetre apres l'echelon : %.0f ms)" % (tau * 1000, t_fin * 1000))
    if t_fin < 4 * tau:
        print("  ATTENTION : fenetre < 4 tau, le palier n'est pas atteint : tau sous-estime.")
    print("  R apparente = V / I = %.2f ohm (inclut la chute du L293D, surestimee)" % R_app)
    if R_mesure:
        print("  ==> L = tau x R = %.2f ms x %.2f ohm = %.1f mH" % (tau * 1000, R_mesure, tau * R_mesure * 1000))
        print("      a reporter dans LM_config.h : LM_COIL_R_OHM %.2ff, LM_COIL_L_H %.4ff"
              % (R_mesure, tau * R_mesure))
    else:
        print("  ==> L ~ %.1f mH avec R apparente ; relancer avec --R <ohmmetre> pour la vraie valeur"
              % (tau * R_app * 1000))
    if plt:
        t = [(k - CAP_PRE) / FS * 1000 for k, r, i in rows]
        fit = [i0 + di * (1 - math.exp(-max(x, 0) / 1000 / tau)) for x in t]
        plt.figure(figsize=(7, 4))
        plt.plot(t, [i for k, r, i in rows], label="courant mesure")
        plt.plot(t, fit, "--", label="ajustement, tau = %.2f ms" % (tau * 1000))
        plt.xlabel("t (ms)"); plt.ylabel("courant (mA)"); plt.grid(alpha=.3); plt.legend()
        plt.title("Echelon de tension, bobine %d" % coil)
        png = os.path.join(dossier, "cap_%d_bobine%d.png" % (num, coil))
        plt.tight_layout(); plt.savefig(png, dpi=120); plt.close()
        print("  -> %s" % png)


def analyse_bode(axe, pts, ident, dossier, num):
    """pts : (f, |T|, phase, Re(1/G) mA/mT, Im(1/G) mA/mT) ; ident : (w0, beta, qualite, ok)."""
    nom = os.path.join(dossier, "bode_%d_axe%d.csv" % (num, axe))
    with open(nom, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["f_Hz", "gain_T", "phase_T_deg", "Re_invG_mA_par_mT", "Im_invG_mA_par_mT"])
        w.writerows(pts)
    print("  -> %s" % nom)
    if ident:
        w0, beta, q, ok = ident
        axe_nom = "X" if axe == 0 else "Y"
        print("  Axe %s : w0 = %.2f rad/s, beta = %.0f mT/(s2.A), qualite = %.3f, %s"
              % (axe_nom, w0, beta, q, "VALIDE" if ok else "NON VALIDE"))
        if ok:
            print("  ==> LM_config.h : LM_W0_%s_RAD_S %.2ff et LM_BETA_%s %.1ff"
                  % (axe_nom, w0, axe_nom, beta))
            print("      ou en direct : lm.id_axis = %d puis lm.cmd = 7" % axe)
        else:
            print("  Modele non valide : w0^2 <= 0 (axe stable ?) ou beta < 0 (verifier coil_sign).")
        if q > 0.3:
            print("  Qualite mediocre (> 0,3) : bruit, saturation, ou amplitude LM_ID_AMP_MT a revoir.")
    if plt and pts:
        f = [p[0] for p in pts]
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        ax[0].semilogx(f, [20 * math.log10(max(p[1], 1e-9)) for p in pts], "o-", label="|T| (dB)")
        ax0b = ax[0].twinx()
        ax0b.semilogx(f, [p[2] for p in pts], "s--", color="tab:orange", label="phase (deg)")
        ax[0].set_xlabel("f (Hz)"); ax[0].set_ylabel("|p / consigne| (dB)"); ax0b.set_ylabel("phase (deg)")
        ax[0].set_title("Boucle fermee mesuree"); ax[0].grid(alpha=.3, which="both")
        w2 = [(2 * math.pi * x) ** 2 for x in f]
        ax[1].plot(w2, [p[3] for p in pts], "o", label="mesure Re(1/G)")
        if ident and ident[1]:
            w0, beta = ident[0], ident[1]
            xx = [0, max(w2) * 1.05]
            ax[1].plot(xx, [-(x + w0 * abs(w0)) / beta * 1000 for x in xx], "--",
                       label="modele : -(w^2 + w0^2)/beta")
        ax[1].set_xlabel("w^2 (rad^2/s^2)"); ax[1].set_ylabel("Re(1/G) (mA/mT)")
        ax[1].set_title("Identification : une droite attendue"); ax[1].grid(alpha=.3); ax[1].legend()
        png = os.path.join(dossier, "bode_%d_axe%d.png" % (num, axe))
        fig.tight_layout(); fig.savefig(png, dpi=120); plt.close(fig)
        print("  -> %s" % png)


# ------------------------------------------------------------------ lecture du flux
class Lecteur:
    def __init__(self, dossier, R_mesure):
        self.dossier, self.R = dossier, R_mesure
        os.makedirs(dossier, exist_ok=True)
        self.journal = open(os.path.join(dossier, "journal.txt"), "w")
        self.fcsv = open(os.path.join(dossier, "donnees.csv"), "w", newline="")
        self.csv = csv.writer(self.fcsv)
        self.csv.writerow(DATA_HEAD)
        self.bloc, self.lignes, self.entete = None, [], None
        self.ncap = self.nbode = 0
        self.ndata = 0
        self.dernier = None
        self.t_aff = 0.0

    def ligne(self, s):
        s = s.strip()
        if not s:
            return
        self.journal.write(s + "\n")
        if s.startswith("#"):
            return self.speciale(s)
        champs = s.split(",")
        if self.bloc == "CAP" and len(champs) == 3:
            self.lignes.append(tuple(int(float(c)) for c in champs)); return
        if self.bloc == "BODE" and len(champs) == 5:
            self.lignes.append(tuple(float(c) for c in champs)); return
        if len(champs) == 10:
            self.csv.writerow(champs); self.ndata += 1; self.dernier = champs
            if time.time() - self.t_aff > 1.0:
                self.t_aff = time.time(); self.etat()

    def etat(self):
        c = [int(x) for x in self.dernier]
        print("  [%6.1f s] mode %-10s defaut %-18s Vbus %5.2f V  px %7.2f  py %7.2f mT  i = %s mA"
              % (c[0] / FS, MODES.get(c[8], c[8]), DEFAUTS.get(c[9], c[9]), c[7] / 1000,
                 c[5] / 100, c[6] / 100, " ".join("%5d" % v for v in c[1:5])))

    def speciale(self, s):
        champs = s[1:].split(",")
        tag = champs[0]
        if tag == "CFG":
            v = champs[1:]
            if v[0] == "0":
                print("\nReglage du correcteur :")
                print("  " + " | ".join(CFG_HEAD))
            print("  " + " | ".join(["X" if v[0] == "0" else "Y"] + v[1:]))
            if v[-1] == "0":
                print("  ATTENTION : reglage incoherent, la levitation sera refusee (defaut 5)")
        elif tag == "CAP":
            self.bloc, self.lignes, self.entete = "CAP", [], (int(champs[1]), int(champs[2]))
        elif tag == "BODE":
            self.bloc, self.lignes, self.entete = "BODE", [], [int(champs[1]), None]
        elif tag == "ID" and self.bloc == "BODE":
            self.entete[1] = (float(champs[2]), float(champs[3]), float(champs[4]), champs[5] == "1")
        elif tag == "END":
            if self.bloc == "CAP":
                self.ncap += 1
                print("\nEssai echelon recu (bobine %d) :" % self.entete[0])
                analyse_cap(self.lignes, self.entete[0], self.entete[1], self.R, self.dossier, self.ncap)
            elif self.bloc == "BODE":
                self.nbode += 1
                print("\nBalayage d'identification recu :")
                analyse_bode(self.entete[0], self.lignes, self.entete[1], self.dossier, self.nbode)
            self.bloc = None

    def fin(self):
        self.journal.close(); self.fcsv.close()
        print("\n%d lignes de donnees enregistrees dans %s" % (self.ndata, self.dossier))


def main():
    ap = argparse.ArgumentParser(description="Outil PC du projet LM")
    ap.add_argument("source", nargs="?", help="port serie (COM5, /dev/ttyACM0) ou journal .txt")
    ap.add_argument("--R", type=float, default=None, help="resistance de la bobine mesuree a l'ohmmetre (ohm)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--ports", action="store_true", help="liste les ports serie")
    ap.add_argument("--dossier", default=None, help="dossier de sortie")
    a = ap.parse_args()

    if a.ports or not a.source:
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                print("%-8s %s" % (p.device, p.description))
        except ImportError:
            print("pyserial absent : pip install pyserial")
        if not a.source:
            ap.print_help()
        return

    dossier = a.dossier or time.strftime("mesures_%Y%m%d_%H%M%S")
    lec = Lecteur(dossier, a.R)
    try:
        if os.path.isfile(a.source):
            with open(a.source, errors="replace") as f:
                for s in f:
                    lec.ligne(s)
        else:
            import serial
            print("Lecture de %s a %d bauds, Ctrl+C pour arreter..." % (a.source, a.baud))
            with serial.Serial(a.source, a.baud, timeout=0.5) as port:
                buf = b""
                while True:
                    buf += port.read(4096)
                    *lignes, buf = buf.split(b"\n")
                    for s in lignes:
                        lec.ligne(s.decode(errors="replace"))
    except KeyboardInterrupt:
        pass
    finally:
        lec.fin()


if __name__ == "__main__":
    main()
