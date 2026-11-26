# ADR 0009: Docking Teil 1 (Wagen aufnehmen und absetzen)

Status: angenommen (Claude, 26.11.2026)

## Inhalt
- Neues Paket `robotros_docking` mit dem Knoten `docking_server` und den Actions `/docking/engage_cart` und
  `/docking/release_cart` (Spezifikation §10, ÄM-01 §3.1/3.2). Logik ohne ROS in `core/maneuver.py`, Tests gegen
  ein einfaches Fahrzeugmodell in `test/test_maneuver.py`.
- Ablauf: Slot-Check am Vorprüfpunkt `P_pre` mit dem ungefilterten Scan (D-37: Abholen braucht mindestens
  `cart_present_min_points` Punkte im Platz, Abstellen höchstens `slot_free_max_points`), Wenden um 180° gegen den
  Uhrzeigersinn, rückwärts einfahren (R1 schnell, R2 geführt auf der Stationsachse), Hub hoch bzw. runter,
  vorwärts `approach_distance` bis `P_a`. Stopp beim Einfahren: Abstand zu `S` ≤ 1 cm, Odometrie-Grenze oder
  Blockade (≤ 4 cm vor `S` = Anschlag, sonst E305/E306). Ohne Kartenpose fährt der Regler nur nach Odometrie.
- Fahrbefehle auf `/cmd_vel_docking` (twist_mux B, vor dem Collision Monitor, wie Spezifikation §3.3), Hub über
  `/hw/cmd/hub`, Summer über `/hw/cmd/buzzer`, Maske über `/mission/scan_mask` (D-30). `scan_masker` maskiert jetzt,
  wenn der Hub oben ist **oder** `/mission/scan_mask` gesetzt ist (bisher nur Hub). Die Maske selbst ist weiter die
  Boxmaske (`BOX`); `SECTOR` folgt.
- Stationsposen liest der Server aus `/web/stations` (vom `mission_manager`).
- `mission_manager`: Ein Slot-Check-Fehler (E301 beim Abholen, E302 beim Abstellen) gilt als „nichts hat sich
  bewegt“: ohne Wagen DEFERRED, mit Wagen HELD_WITH_CART – nicht NEEDS_OPERATOR.

## Abweichungen und Festlegungen
- **`reverse_k_y` = 8,0 statt Startwert 1,5** (ÄM-01 §5, Quelle „K, C15“): Mit 1,5 baut der Regler 4 cm
  Seitenversatz erst nach etwa 1,3 m ab (Kennlänge `k_θ/k_y`), die Einfahrt ist aber nur 0,70 m lang. Mit 8,0 sind
  es etwa 0,25 m. Festzulegen in Kalibrierung C15.
- **`slot_margin_m` = 0 statt Startwert 0,04 m:** Platz S3 der Simulationswelt liegt 0,25 m vor der Wand; mit
  0,24 m halber Platzgröße und einigen Zentimetern Posenfehler zählte die Wand als „belegt“ (E302). Mit 0,20 m
  liegen die Wagenfüße (±0,19 m) noch im Platz. Am echten Aufbau prüfen.
- **Blockiererkennung über die Geschwindigkeit aus `/odom`** (`stall_speed` 0,005 m/s) statt `stall_tps`
  (5 Ticks/s ≈ 0,002 m/s mit den Platzhaltern). Am echten Roboter prüfen.
- Drehrichtung beim Wenden immer gegen den Uhrzeigersinn; die Seitenwahl nach Kosten der lokalen Costmap fehlt.
- Abbruch in R1 durch den hinteren Ultraschall, Orange-Lauflicht und Feinjustage (Nudge) fehlen noch.
- `footprint_switcher` und Geschwindigkeitsprofil CART fehlen noch: Nav2 plant mit Wagen weiter mit dem
  Roboter-Footprint.

## Simulation
Die Wagen in Gazebo sind noch statisch: Nach dem „Anheben“ bleibt der Wagen stehen, nur die Datenbank führt ihn als
transportiert. Die Prüfung `mission_check.py` (CI) fährt einen echten Transport von S1 (Wagen C1 mit Füßen, 0,36 m
lichte Weite bei 0,30 m Roboterbreite) zum freien Platz S3 und prüft, dass der Roboter bis auf 5 cm an S1 heranfährt,
der Auftrag COMPLETED ist und der Wagen in der Datenbank bei S3 steht. Das Mitnehmen des Wagens in Gazebo folgt.
