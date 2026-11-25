# ADR 0003: Durchfahrt durch Engstellen

Status: angenommen (Claude, 25.11.2026; Softwaredetail nach Runde 25)

## Anlass
Hannes (25.11.2026, nach PR #16): Der Roboter schafft den engen Durchgang der Simulationswelt (0,45 m,
Spec §14) überhaupt nicht; die Karte „springt“ während der Fahrt sichtbar ein wenig hin und her.
Spec §7.2: kleinste befahrbare Gasse = Footprintbreite + 0,10 m, also 0,40 m bei 0,30 m Breite.

## Messungen (CI, Gazebo; `sim_nav_check.py` gibt MEASURE-Zeilen aus; Probe-Branch `feature/passage-probe`)
Durchgang: Ziel B (0,7; −1,5) ↔ P (2,5; −0,775) mit scharfer Kurve hinter dem Durchgang, und W (0,5; −0,775)
↔ P geradeaus. „Durchfahrten“ = gelungene Fahrten durch den Durchgang.

| Einstellung | Ergebnis |
|---|---|
| bisher (Vorausschau fest 0,40 m, Padding 0,01 m, lokale Costmap 5 cm) | 0 von 3 Versuchen, jedes Mal „collision ahead“ des Reglers (RPP) |
| + Vorausschau geschwindigkeitsabhängig, Padding 0 | 7 von 8 bzw. 1 von 4, mehrmals festgeklemmt im Durchgang |
| + zusätzlich lokale Costmap 2,5 cm | **gerade 7/7, scharfe Kurve 7/7** |
| wie oben, aber Planer `cost_travel_multiplier` 3,0 | schlechter (gerade 6/7 mit Abbruch, scharf 3/7) |

Ursachen: Der Regler sah 0,40 m voraus und kürzte die Einfahrt ab (bis 5 cm neben der Mitte). Bei
7,5 cm Luft je Seite und Zellen von 5 cm meldete seine Kollisionsprüfung dann eine Berührung, obwohl
real noch Platz war. Mit 2,5-cm-Zellen halbiert sich dieser Rundungsfehler.

AMCL („Springen“): Korrektur je Update im Median 3–9 mm, 90 % unter 2 cm, einzelne bis 7 cm. Das ist die
normale Arbeitsweise des Partikelfilters. Mehr Strahlen (180) und `sigma_hit` 0,1 verschlechterten in
der Messung das Endergebnis an Ziel B (M1-03 6/10 statt 10/10). Deshalb bleiben die Nav2-Standardwerte.

## Entscheidung
1. Regler: `use_velocity_scaled_lookahead_dist: true`. Die Spec-Werte bleiben unverändert (0,40 / 0,20 /
   0,60 m); die Grenzen min/max wirken in Nav2 nur so.
2. `footprint_padding: 0.0` in beiden Costmaps (Nav2-Standard 0,01 m).
3. **Planer `cost_travel_multiplier` 3,0 statt 2,0 (Abweichung von Spec §7.2)**, siehe Nachmessung unten.
4. **Lokale Costmap 0,025 m statt 0,05 m (Abweichung von Spec §7.2).** Bei 3 × 3 m sind das 120 × 120 Zellen.
   Die globale Costmap und die Karte bleiben bei 0,05 m.
5. Der Collision Monitor prüft unverändert mit dem echten Footprint gegen die Rohdaten des Lidar.
6. Test: Die Kartierung in `tools/sim_nav.sh` fährt jetzt eine Runde durch den ganzen Raum und durch den
   Durchgang (Testfahrer auf der Ground Truth). Die Navigationsphase benutzt die exakte Karte der Welt
   (`world_map.py`), damit die Endfehler nur Navigation und AMCL messen und nicht die Qualität eines
   einzelnen SLAM-Laufs. Die SLAM-Qualität schwankte zwischen Läufen: die Endpunkte lagen je Karte
   gleichbleibend um 2–13 cm versetzt, bei einer Streuung innerhalb einer Karte von etwa 2 cm.
   Die Navigation fährt A, B und P (durch den Durchgang von West nach Ost). Die Fahrt durch den Durchgang
   ist in der CI vorerst nur informativ (WARN statt FAIL), weil sie noch nicht immer gelingt.

## Nachmessung mit allen Änderungen (Probe-Lauf, exakte Karte bzw. SLAM-Karte)
- M1-03 auf der exakten Karte: 10/10 (Endfehler 0,035–0,084 m, 2,4–5,0°).
- West → Ost durch den Durchgang: 4/4 (exakte Karte), 3/3 (SLAM-Karte).
- Ost → West mit scharfer Kurve hinter dem Durchgang (P → B): 1/4 auf der exakten Karte, 3/3 auf der
  SLAM-Karte. In den Fehlschlägen führte der geplante Weg bis 12,5 cm neben der Mitte durch den Durchgang
  (der Planer kürzt Richtung B ab); der Roboter blieb davor stehen.
- Weg mittiger halten, je 11 Fahrten abwechselnd P ↔ B auf der exakten Karte (Durchgang in beide Richtungen):
  | Planer | gelungene Durchfahrten vor dem ersten Abbruch | Abbrüche gesamt |
  |---|---|---|
  | wie bisher (`cost_travel_multiplier` 2,0) | 1 | 7 |
  | `cost_travel_multiplier` 3,0 | **9** | 2 |
  | Glättung schwächer (w_smooth 0,1 / w_data 0,4) | 0 | 8 |
  | beides | 8 | 3 |
  Abbrüche nach dem ersten Festklemmen folgen teils daraus, dass der Roboter schon im Durchgang steht.
  **Offen:** auch mit 3,0 nicht immer zuverlässig. Nächste Kandidaten: Recovery, die im Durchgang
  geradeaus zurücksetzt; Footprint-Prüfung des Reglers gegen die gröbere Rundung; Engstelle als eigener Modus.

## Folgen und Grenzen
- Kleine Wackler (unter 1–2 cm) der Lokalisierung bleiben sichtbar.
- Bleibt der Roboter doch im Durchgang hängen, kann die Recovery (ADR 0002) nicht zurücksetzen, wenn auch
  hinten die Costmap eine Berührung meldet. Dann bricht der Auftrag ab (E204), der Roboter steht.
- Am echten Roboter ist die Rechenlast der feineren lokalen Costmap auf dem Pi ungeprüft.
- Die Lage der Karte gegenüber der echten Welt hängt von der Kartierfahrt ab. Für Stationen ist das
  unkritisch, weil sie in der Karte eingelernt werden (D-33).
