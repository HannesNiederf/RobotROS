# ADR 0003: Durchfahrt durch Engstellen und ruhigere Lokalisierung

Status: angenommen (Claude, 25.11.2026; Softwaredetail nach Runde 25)

## Anlass
Hannes (25.11.2026, nach PR #16): Der Roboter schafft den engen Durchgang der Simulationswelt (0,45 m,
Spec §14) überhaupt nicht; die Karte „springt“ während der Fahrt sichtbar ein wenig hin und her.
Spec §7.2: kleinste befahrbare Gasse = Footprintbreite + 0,10 m, also 0,40 m bei 0,30 m Breite.

## Messungen (CI, Gazebo, `sim_nav_check.py` mit MEASURE-Zeilen, Probe-Branch `feature/passage-probe`)
- Mit den bisherigen Werten brach jede Fahrt durch den Durchgang ab: der Regler (RPP) meldete
  „collision ahead“. Der Roboter fuhr bis zu 5 cm neben der Mitte ein. Bei 7,5 cm Luft je Seite und
  Kostkartenzellen von 5 cm reicht das für eine Kollisionsmeldung.
- Ursache des Versatzes: der Pure-Pursuit-Regler schaute fest 0,40 m voraus und kürzte die Einfahrt in den
  Durchgang ab. Die Spec nennt dazu min 0,20 / max 0,60 m; diese Grenzen wirken in Nav2 nur mit
  `use_velocity_scaled_lookahead_dist: true`.
- Mit geschwindigkeitsabhängigem Vorausschauen (langsam in engen Stellen → 0,20 m) kam der Roboter
  durch den Durchgang, solange die Lokalisierung stimmte. Die verbliebenen Fehlschläge hatten
  AMCL-Fehler von 8–10 cm in einem Bereich, den die CI-Karte kaum enthielt (Karte nur aus der Raummitte).
- AMCL-Korrekturen je Update (das sichtbare „Springen“): Median 4–9 mm, 90 % unter 2 cm, einzelne bis 7 cm.
  Mit 180 statt 60 Strahlen und `sigma_hit` 0,1 statt 0,2 sank der mittlere AMCL-Fehler von 12–33 mm auf
  6–15 mm (je Variante eine Fahrt, Tendenz, keine Statistik).
- Feineres Raster der lokalen Costmap (2,5 cm) brachte in der Messung keinen klaren Vorteil → bleibt bei 5 cm (Spec).

## Entscheidung
1. Regler: `use_velocity_scaled_lookahead_dist: true` (Werte der Spec unverändert: 0,40 / 0,20 / 0,60 m).
2. `footprint_padding: 0.0` in beiden Costmaps (Nav2-Standard 0,01 m). Der Collision Monitor prüft weiter
   mit dem echten Footprint gegen die Rohdaten des Lidar.
3. AMCL: `max_beams` 180, `sigma_hit` 0,1 (nicht von der Spec festgelegt; Spec-Werte wie Partikelzahl und
   alpha bleiben).
4. Test: Die Kartierung in `tools/sim_nav.sh` fährt jetzt eine Runde durch den ganzen Raum und durch den
   Durchgang (Testfahrer auf der Ground Truth). Die Navigation fährt zusätzlich B → P (hinter dem Durchgang)
   → B und verlangt, dass der Durchgang benutzt wird.

## Folgen und Grenzen
- Kleine Wackler (unter 1–2 cm) beim Lokalisieren bleiben normal, sie sind die Arbeitsweise von AMCL.
- Eine gute Karte ist Voraussetzung: beim Kartieren auch durch Engstellen und in alle Ecken fahren.
- Am echten Roboter mit Messung prüfen (Lidar-Rauschen, Rechenzeit von 180 Strahlen auf dem Pi ungeprüft).
