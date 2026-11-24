# ADR 0002: Recovery im Behavior Tree setzt zurück; Odometrie der Simulation kalibriert

Status: angenommen (Claude, 25.11.2026; Softwaredetail nach Runde 25, „Claude entscheidet alle Softwaredetails“)

## Anlass
Erster Test von Hannes mit `tools/sim_start.sh nav` (24.11.2026, Bildschirmfoto in RViz):
- Beim Drehen lag der Lidar-Scan verdreht zur Karte, in der globalen Costmap entstanden Scheinhindernisse,
  der Roboter „schloss sich virtuell ein“.
- Der Roboter blieb in einer Durchfahrt stecken und setzte nicht von selbst zurück.

## Befund 1: Odometrie der Simulation
Messung in der CI (`sim_start_check.py`, Gazebo Harmonic, 25.11.2026): Drehung auf der Stelle mit 0,6 und
0,3 rad/s – die Radodometrie (`/odom`, DiffDrive-Plugin) meldet 360°, der Roboter hat sich laut Ground Truth
nur um 312,6° gedreht (Fehler 13 %). Geradeausfahrt: 0,450 m zu 0,450 m. Die Räder rutschen in Gazebo beim
Drehen. Ein so großer Drehfehler übersteigt, was AMCL während der Fahrt ausgleicht; die Pose wird erst im
Stillstand korrigiert. Dazwischen markiert die Costmap Wände an falscher Stelle.

Entscheidung: Die Simulation bekommt eine Kalibrierung wie der echte Roboter (C3, wirksame Spurweite):
`robotros_sim/config/sim_hw.yaml` → `gazebo_track_scale: 1.152`. Nur das DiffDrive-Plugin rechnet damit, die
Radpositionen im Modell bleiben bei der echten Spurweite. `sim_start_check.py` (CI) prüft dauerhaft, dass
der Drehfehler ≤ 3 % bleibt. Der echte Roboter wird nach `spez_parameter_tests.md` C3 kalibriert
(`track_scale` in `robot.yaml`).

## Befund 2: kein Zurücksetzen bei Festfahren
Spezifikation §7.4: Recovery = warten (4 s) und lokale Costmap löschen, bis zu 3 Wiederholungen. Steht der
Roboter dicht vor einer Wand oder in einer Engstelle, plant er danach denselben Weg und kommt nicht frei.

Entscheidung: Der Recovery-Zweig ist jetzt `Wait` (4 s) → `ClearEntireCostmap` (lokal) → `BackUp`
0,10 m mit 0,05 m/s (`recovery_backup_m`, D-29 erlaubt Rückwärtsfahren in der Recovery mit 0,05 m/s).
- `BackUp` prüft die lokale Costmap (Lidar sieht ohne Wagen 360°) und fährt durch Velocity Smoother,
  twist_mux und Collision Monitor; die Sicherheitslogik des Nano bleibt unverändert aktiv.
- Kann er nicht zurücksetzen, läuft die Wiederholung ohne Zurücksetzen weiter (`ForceSuccess`).
- **Mit Wagen** (Sektormaske, hinten blind) gilt ÄM-01: nur 0,05 m. Das muss die Mission/der
  `footprint_switcher` umschalten, sobald es Wagenfahrten gibt (offen; bis dahin gibt es keine).

## Folgen
- §7.4 der Softwarespezifikation ist um das Zurücksetzen erweitert (diese ADR gilt vorrangig).
- Ungeprüft: ob 0,10 m in allen Engstellen reicht; ob das Rutschen in Gazebo bei anderen Böden/Geschwindigkeiten
  anders ausfällt.
