# Ergebnis-Zusammenfassung: Probabilistic Robotics Project (2510331007 – Wrong Initialization)

Alle Werte stammen aus echten Simulationsläufen (TurtleBot4, Gazebo, ROS2 Jazzy),
Trajektorie: `circle`, v=0.2 m/s, w=0.3 rad/s, num_particles=500 (PF).

---

## 1. Wrong-Initialization-Experimente (Kernaufgabe)

| Szenario | Beschreibung | init_x, init_y | init_cov (KF/EKF) / init_spread (PF) |
|---|---|---|---|
| W0 | Baseline, korrekte Init | 0, 0 | klein (0.01 / 0.1) |
| W2 | Großer Init-Fehler | 1.0, 2.0 | moderat (0.5 / 0.3) |
| W3 | Großer Init-Fehler + überzeugt | 1.0, 2.0 | sehr klein (0.001 / 0.05) |

### Konvergenzzeit t_conv [s] (Fehler < 0.2 m, sustained 2s)

| Filter | W0 | W2 | W3 | Faktor W3/W0 |
|---|---|---|---|---|
| KF  | 1.00 | 1.15 | **5.10** | **5.1x** |
| EKF | 1.00 | 0.55 | 2.65 | 2.65x |
| PF  | 1.00 | 25.15 | 26.60 | **26.6x** |

### RMSE_pos [m]

| Filter | W0 | W2 | W3 |
|---|---|---|---|
| KF  | 0.0007 | 0.0039 | 0.0607 |
| EKF | 0.0142 | 0.0153 | 0.0126 |
| PF  | 0.4301 | 0.3683 | 0.3180 |

**Kernbefund:** Der Initialisierungsfehler wird umso stärker "mitgezogen"
(= langsamere Konvergenz), je überzeugter (geringere Anfangsunsicherheit)
der Filter von seiner falschen Anfangsannahme ist. Der Effekt ist am
stärksten beim Partikelfilter (Faktor ~27x), gefolgt vom klassischen
Kalman-Filter (Faktor ~5x), während der EKF durch sein volles
Pose-Messmodell (aggressive Korrektur bei jeder Messung) am robustesten
gegenüber der Initialisierung bleibt.

---

## 2. Q-Variation (Prozessrauschen), R fixiert auf 0.05, Init korrekt (0,0)

### RMSE_pos [m]

| Filter | Q=0.001 | Q=0.01 | Q=0.1 | Q=1.0 |
|---|---|---|---|---|
| KF  | 0.0012 | 0.0005 | 0.0002 | 0.0001 |
| EKF | 0.0310 | 0.0117 | 0.0026 | 0.0004 |
| PF  | **2.4955** | 1.8948 | 0.4130 | **0.0255** |

**Kernbefund:** Bei sehr kleinem Prozessrauschen (Q=0.001) divergiert der
PF vollständig (RMSE > 2 m), weil die enge Partikelstreuung keine Anpassung
an reale Bewegungsabweichungen erlaubt. Mit steigendem Q verbessert sich
die Schätzung aller drei Filter monoton; der Effekt ist beim PF mit
Faktor ~98 zwischen den Extremwerten am dramatischsten. KF/EKF sind
durchgehend deutlich robuster gegenüber dieser Parameterwahl.

---

## 3. R-Variation (Messrauschen), Q fixiert auf 0.01, Init korrekt (0,0)

### RMSE_pos [m]

| Filter | R=0.001 | R=0.05 | R=0.5 | R=2.0 |
|---|---|---|---|---|
| KF  | 0.0001 | 0.0007 | 0.0021 | 0.0033 |
| EKF | 0.0008 | 0.0136 | 0.0637 | 0.1283 |
| PF  | 0.0703 | **0.0280** | 0.3427 | **1.3862** |

### Konvergenzzeit t_conv [s]

| Filter | R=0.001 | R=0.05 | R=0.5 | R=2.0 |
|---|---|---|---|---|
| KF  | 1.40 | 0.65 | 2.80 | 2.70 |
| EKF | 1.40 | 0.65 | 2.80 | 2.90 |
| PF  | 2.15 | 0.65 | 2.80 | **nan (keine Konvergenz)** |

**Kernbefund:** KF/EKF zeigen das erwartete monotone Verhalten – größeres R
(weniger Vertrauen in die Messung) führt zu größerem Fehler, da das
simulierte Odometrie-Rauschen tatsächlich gering ist und daher starkes
Messvertrauen meist vorteilhaft ist. Der PF zeigt dagegen einen Sweet Spot
bei R=0.05: zu enges R (0.001) führt zu Partikel-Degeneration (zu strenge
Likelihood), zu weites R (2.0) lässt den Filter komplett divergieren
(keine Konvergenz innerhalb der Messzeit).

---

## 4. Runtime / Performance

| Filter | Update-Rate [Hz] | Kumulierte CPU-Zeit [s] |
|---|---|---|
| KF  | ~12.1 | 1.46 |
| EKF | ~11.5 | 1.70 |
| PF (N=500) | ~11.3 | 2.05 |

**Kernbefund:** Alle drei Filter erreichen vergleichbare Update-Raten
(~11-12 Hz), limitiert durch die Odometrie-Publish-Frequenz, nicht durch
die eigene Rechenzeit. Die kumulierte CPU-Last steigt jedoch in der
erwarteten Reihenfolge KF < EKF < PF entsprechend der Modellkomplexität
(Jacobi-Matrix beim EKF, partikelbasierte Likelihood-Berechnung und
Resampling beim PF).

---

## 5. Gesamtfazit (für Diskussion/Conclusion im Paper)

1. **KF** ist am einfachsten und schnellsten, liefert aber nur bei linearen
   Bewegungsmodellen exakte Ergebnisse; zeigt moderate Empfindlichkeit
   gegenüber überzogener Anfangsunsicherheit (W3).
2. **EKF** ist über alle Experimente hinweg der robusteste Filter:
   beste Balance aus Genauigkeit, Rechenaufwand und Robustheit gegenüber
   Initialisierungsfehlern.
3. **PF** liefert bei guter Parametrisierung (insb. ausreichendes
   Prozessrauschen, moderates R, ausreichender init_spread) konkurrenzfähige
   Ergebnisse, ist aber in jedem der drei Pflicht-Experimente (Wrong-Init,
   Q-Variation, R-Variation) der mit Abstand empfindlichste und am
   schwierigsten zu parametrisierende Filter. Dies bestätigt ein bekanntes
   Ergebnis aus der probabilistischen Robotik: PF-Performance hängt stark
   von einer sorgfältigen Parameterwahl ab, während KF/EKF von Natur aus
   robuster gegenüber Fehlparametrisierung sind.
