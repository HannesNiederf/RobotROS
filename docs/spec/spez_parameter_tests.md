# Parameterkatalog, Kalibrierung, Tests und Abnahme „robotROS“ v1.0

Stand: 18.11.2026 · Teil der Softwarespezifikation (`claude/softwarespezifikation.md`, `claude/spez_firmware_protokoll.md`)
Zweck: Jeder bisher offene Zahlenwert ist hier ein **Parameter mit Startwert, Einheit und Kalibrierverfahren**. Startwerte sind Platzhalter, bis gemessen wurde. Sie stehen in YAML-Dateien (Paket `robotros_bringup/config/`), nie im Code. Spalte „Quelle“: **K** = Kalibrierung (Abschnitt 3), **CAD** = aus der Konstruktion, **T** = Test/Feintuning, **E** = Entscheidung (fest).

---

## 1. Dateien

| Datei | Inhalt |
|---|---|
| `robot.yaml` | Geometrie, Montagepositionen, Odometrie |
| `firmware.yaml` | alle `$CFG`-Werte (Spec Firmware §10) |
| `speed_profiles.yaml` | Geschwindigkeitsprofile |
| `power.yaml` | Akku-/Ladeschwellen |
| `mission.yaml` | Aufträge, Lokalisierung, Docking |
| `obstacles.yaml` | Kontaktpunkte, Sperrzonen, Masken |
| `nav2.yaml`, `slam.yaml`, `amcl.yaml` | Navigation (Werte in Spec 1 §7) |
| `bridge.yaml` | Geräteliste, Raten |

## 2. Parameterkatalog

### 2.1 `robot.yaml`
| Parameter | Einheit | Startwert | Quelle |
|---|---|---|---|
| `robot_length`, `robot_width` | m | 0,26 / 0,26 (Prüfung REQ-MECH-05, vermutlich ≤ 0,20 nötig) | CAD |
| `cart_size` | m | 0,30 (Außenmaß) | CAD |
| `footprint_robot` | m | Quadrat 0,26 | CAD |
| `footprint_cart` | m | Quadrat 0,32 | CAD |
| `wheel_radius_m` | m | 0,0325 | CAD, K (C2) |
| `track_width_m` | m | 0,16 | CAD, K (C3) |
| `ticks_per_wheel_rev` | Ticks | 576 (Platzhalter) | K (C1) |
| `wheel_radius_scale`, `track_scale` | – | 1,0 | K (C2, C3) |
| `geometry_calibrated` | bool | `false` | K (C1–C3) |
| `lidar_x`, `lidar_z` | m | 0,0 / 0,12 | CAD |
| `lidar_yaw_rad` | rad | 0,0 | K (C7) |
| `us_front_x`, `us_rear_x` | m | +0,13 / −0,13 | CAD |
| `us_pitch_deg` | ° | 45 | CAD |
| `us_ground_x` | m | 0,21 (Bodenpunkt vor der Mitte) | CAD |
| `bumper_front_x` | m | 0,14 | CAD |
| `self_mask_boxes` | m | leer | CAD, K (C7) |
| `serial_latency_s` | s | 0,005 | T |
| `odom_cov_xy`, `odom_cov_yaw` | – | 0,02 / 0,05 | T |

### 2.2 `speed_profiles.yaml`
| Profil | `v` m/s | `ω` rad/s | Bemerkung |
|---|---|---|---|
| NORMAL | 0,15 | 0,6 | Fahrt ohne Wagen |
| CART | 0,12 | 0,5 | Fahrt mit Wagen |
| DOCK | 0,05 | 0,3 | Einfahren, Andocken, Ausfahren |
| MANUAL | 0,20 | 0,8 | Joystick |
| MANUAL_SLOW | 0,10 | 0,4 | Joystick, Touch-Mapping |
Alle Werte ≤ `max_speed` aus dem Lasttest (C10). Die Bridge begrenzt zusätzlich auf 0,05 m/s bei `geometry_calibrated: false`.

### 2.3 `mission.yaml`
| Parameter | Einheit | Startwert | Quelle |
|---|---|---|---|
| `person_wait_s` | s | 4,0 (Bereich 3–5) | E, T |
| `nav_timeout_s` | s | 180 | T |
| `nav_retries` | – | 3 | E |
| `contact_retries` | – | 3 | E |
| `recovery_backup_m` | m | 0,10 | T |
| `approach_distance` | m | 0,40 | CAD, T |
| `engage_speed` | m/s | 0,05 | E |
| `engage_window_early_m` / `_late_m` | m | 0,08 / 0,03 | T |
| `engage_tolerance_xy`, `_yaw` | m, rad | 0,05 / 0,10 | T |
| `hub_timeout_s` | s | 8,0 | K (C12) |
| `undock_travel_m` | m | 0,25 | T |
| `dock_retries` | – | 3 | E |
| `charger_detect_s` | s | 3 | T |
| `mains_poll_s` | s | 10 | E |
| `loc_wait_s` | s | 5 | T |
| `loc_sigma_xy_ok`, `loc_sigma_yaw_ok` | m, rad | 0,10 / 0,15 | T |
| `loc_lost_sigma`, `loc_lost_s` | m, s | 0,25 / 5 | T |
| `rotate_speed`, `rotate_max_deg` | rad/s, ° | 0,3 / 720 | E |
| `cart_check_enabled` | bool | `false` | E (M3 optional) |
| `queue_default_priority` | 0–9 | 5 | E |
| `system_dock_priority` | 0–9 | 0 | E |

### 2.4 `obstacles.yaml`
| Parameter | Einheit | Startwert |
|---|---|---|
| `contact_lifetime_s` | s | 120 |
| `contact_merge_radius` | m | 0,15 |
| `contact_promote_hits` | – | 3 |
| `contact_zone_width` | m | 0,09 |
| `bumper_zone_y` | m | 0,08 |
| `us_obstacle_lifetime_s` | s | 30 |
| `cliff_zone_width_margin` / `_depth` | m | 0,10 / 0,20 |
| `cliff_auto_zone_max` | – | 30 |
| `cart_mask_box` / `cart_mask_margin` | m | ±0,17 / 0,03 |
| `scan_range_min`, `scan_range_max` | m | 0,15 / 8,0 |

### 2.5 `power.yaml` (alle bis zum Akkutest C9 Platzhalter)
| Parameter | Einheit | Startwert |
|---|---|---|
| `low_battery_v` | V | 11,8 |
| `critical_v` | V | 11,2 |
| `shutdown_v` | V | 10,8 |
| `min_order_v` | V | 11,8 |
| `recharge_v` | V | 12,6 |
| `charger_present_v` | V | 12,0 |
| `charge_detect_ma` | mA | 50 |
| `charge_end_ma` / `charge_end_hold_s` | mA, s | 50 / 60 |
| `charge_end_min_v` | V | 13,5 |
| `charge_max_h`, `charge_max_h_no_ina` | h | 8, 4 |
| `voltage_gain_*`, `voltage_offset_*` | – | 1,0 / 0,0 (C8) |
| `divider_ratio` | – | 5,0 [DB: 30 k/7,5 k] |

### 2.6 Firmware (`firmware.yaml`) siehe Spec Firmware §10.

---

## 3. Kalibrierverfahren

Jedes Verfahren: Werkzeug (`tools/…`), Durchführung, Ergebnis (Parameter), Abnahme. Ergebnisse im Testprotokoll festhalten (Vorlage Abschnitt 7) und in die YAML-Dateien eintragen.

| ID | Thema | Durchführung | Ergebnis / Abnahme |
|---|---|---|---|
| **C1** | Ticks je Radumdrehung | Rad mit Markierung genau 10 Umdrehungen von Hand drehen (Motor gebremst, Roboter aufgebockt), `tools/calib_ticks.py` zählt Ticks. 3 Wiederholungen je Rad. | `ticks_per_wheel_rev` = Mittelwert/10. Abweichung links/rechts ≤ 1 %, sonst Encoder prüfen. |
| **C2** | Raddurchmesser / Geradeausfahrt | 2,00 m Strecke markieren, Roboter fährt per `tools/drive_straight.py` mit 0,10 m/s, 5 Läufe. Vergleich Odometrie mit Messung. | `wheel_radius_scale` = gemessen/odom. Abnahme: Streuung ≤ 2 %, seitlicher Versatz auf 2 m ≤ 5 cm. |
| **C3** | Spurbreite | Auf der Stelle 10 volle Drehungen (Markierung am Boden), `tools/drive_rotate.py`. | `track_scale` so, dass odometrischer Winkel = 3600° ± 2 %. Danach `geometry_calibrated: true`. |
| **C4** | Regler | `tools/pid_tune.py`: Sprünge 0 → 40 % → 80 % des `TPS_MAX`, beide Räder aufgebockt, dann am Boden. Erst `KFF` aus (PWM beim stationären Sollwert / Sollwert), dann `KP` = 0,5·`KFF`, `KI` = 2·`KFF` je s, `KD` = 0. | Abnahme: Anstiegszeit ≤ 0,3 s, Überschwingen ≤ 10 %, stationärer Fehler ≤ 3 % Ticks/s. |
| **C5** | PWM-Totzone | Sollwert langsam von 0 erhöhen, PWM bei der ersten Raddrehung je Seite notieren (`tools/deadband.py`). | `PWM_MIN_L/R` = gemessen + 10 %. |
| **C6** | Ultraschall und Bremsweg | (a) Auf dem Einsatzboden: Abstand zum Boden bei der Montage je Sensor 20-mal messen → `US_FLOOR_F/R` = Mittelwert, Streuung ≤ 5 mm. (b) Kantentest: Roboter fährt mit 0,15 m/s auf eine Kante zu (Tischkante im Abstand mit Fangnetz, Roboter angeleint, Wagen weg): 10 Läufe je Sensor, Haltepunkt zur Kante messen. | Abnahme: Halt ≥ 3 cm **vor** dem ersten Bodenkontakt-Verlust, 10/10, Bremsweg-Messung ersetzt Annahme 2 cm in Spec Firmware §6.3. Bei Fehlern `US_CLIFF_MARGIN`/Montage anpassen. |
| **C7** | Lidar-Montage | Roboter 1,0 m parallel vor einer geraden Wand: Winkelversatz aus dem Scan (`tools/lidar_align.py`). Ränder der Selbstverdeckung ermitteln. | `lidar_yaw_rad` ≤ 0,5° Restfehler, `self_mask_boxes` eintragen. |
| **C8** | Spannungssensoren | Mit Multimeter bei 2 Spannungen (z. B. 12,0 V und 13,8 V am Netzteil) vergleichen, je Sensor. | `voltage_gain/offset` = Gerade durch beide Punkte. Abnahme: Fehler ≤ 50 mV. |
| **C9** | Akkutest | (1) Akku voll laden (Ladegerät 13,8 V, bis Strom < 50 mA). (2) `tools/battery_profile.py` fährt ein festgelegtes Lastprofil (Pi, Lidar, Rechteckkurs mit 0,15 m/s, alle 5 min Hub zyklisch) und loggt `v_batt` alle 10 s bis 11,0 V unter Last. (3) Ruhespannung nach 30 min Pause bei fünf Entladestufen. (4) Ladeprofil (Strom über die Zeit) beim Wiederaufladen. | Ergebnis: Laufzeit bis `low_battery_v` und `critical_v`, Ruhespannung-zu-Ladezustand-Tabelle, typische Ladedauer (erwartet ca. 3,5 h bei 0,7 A [Rechnung aus Datenblatt]), `charge_end_ma`. Alle Werte in `power.yaml`. `low_battery_v` so, dass die Restlaufzeit für den schlimmsten Fahrweg zum Dock plus 10 min reicht. |
| **C10** | Motor-Lasttest | Drei Lasten: Roboter leer, mit leerem Wagen, mit Wagen + 1 kg. Je: 3 m geradeaus, 360° Drehung, Überfahren einer 3-mm-Kante, 5 min Dauerlauf. Messen: erreichte Geschwindigkeit, `v_rail` Minimum, PWM-Auslastung, Temperatur des L298N-Kühlkörpers. | Abnahme: ≥ 0,10 m/s und ≥ 0,4 rad/s mit Wagen + 1 kg, `v_rail` ≥ 4,6 V, kein Stillstand, Temperatur ≤ 70 °C. **Sonst Fallback-Kette (Abschnitt 4).** Danach `speed_profiles.yaml` und `PWM_MAX`/`TPS_MAX` festlegen. |
| **C11** | Stationen/Dock einlernen | Per UI (Spec 1 §8.4). Danach 10 Anfahrten aus verschiedenen Startposen. | Abnahme siehe M2-01 und M3-02. |
| **C12** | Hubzeit | `tools/hub_cycle.py`: 10 Zyklen hoch/runter, ohne und mit Wagen. | `hub_timeout_s` = 1,5 × längste Zeit, `HUB_PWM` so, dass Seil/Winde ruhig laufen. |
| **C13** | Bumper | Jede Zone mit einer Wand 10-mal antasten bei 0,05 m/s. | Abnahme: Zonenzuordnung 10/10 richtig, Kontaktpunkt in der Karte ≤ 0,10 m neben dem wahren Punkt. |

---

## 4. Fallback-Kette für die Motorversorgung (D-25)

Ausgelöst, wenn C10 fehlschlägt. Jede Stufe wird erst eingesetzt, wenn die vorherige nicht genügt.

| Stufe | Maßnahme | Software-Folge |
|---|---|---|
| 0 | Plan: 5-V-Sicherheitsschiene, L298N, `5V_EN` entfernt [Nutzer, Runde 24] | – |
| 1 | Schiene verstärken: 1000 µF + 100 nF direkt an der L298N, Leitungen kurz und dick, Wandler ≥ 3 A. Profile auf 0,10 m/s absenken. | nur YAML |
| 2 | **Motorschiene höher:** getrennter einstellbarer Wandler 7,2 V für die Sicherheitsschiene (die L298N verliert ca. 2 V, Motor sieht ca. 5,2 V, innerhalb 3–9 V). | `PWM_MAX` ggf. begrenzen |
| 3 | **Treiber tauschen:** MOSFET-Brücke (z. B. TB6612FNG oder DRV8833) statt L298N, Spannungsabfall ca. 0,5 V statt 2 V, Motor sieht bei 5 V ca. 4,5 V. Ansteuerung bleibt IN1/IN2/PWM. [Wissen, Datenblätter vor Kauf prüfen] | Pin-Header, ggf. Bremsverhalten |
| 4 | **Mechanik:** kleinere Räder (mehr Drehmoment, weniger Geschwindigkeit) oder anderes Getriebe. | `wheel_radius_m`, Profile |
Bei einer Motorspannung von 12 V (Stufe 3 ohne Regler) begrenzt `PWM_MAX` die Spannung am Motor auf ≤ 6 V (Faustformel Duty ≈ 6 V / (12 V − Abfall)). Der Hubmotor (ROB-13260) wird getrennt gemessen (C12).

---

## 5. Teststrategie

| Ebene | Inhalt | Wo | Wann |
|---|---|---|---|
| L0 Unit | Firmware-Kern (native), Python-Kernlogik (`core/*.py`), Protokoll mit goldenen Vektoren, i18n-Prüfung | CI und lokal | bei jedem Commit |
| L1 Bridge/Protokoll | `nano_bridge` gegen `fake_nano` (pty), Fehlerfälle (Prüfsumme, Reset, Verlust) | CI | jeder Commit |
| L2 Simulation | `launch_testing` in Gazebo: M1/M2/M3-Szenarien automatisiert | Entwicklungsrechner/CI (nightly) | jede Woche |
| L3 HIL | Nano Every mit Ersatzaufbau am Pi, `tools/hil_smoke.py` | Pi | nach Firmwareänderung |
| L4 Real | Abnahmetests am Roboter mit Messprotokoll | Raum | je Meilenstein |
Regeln: Jede Zeile der Sicherheitstabelle (Spec Firmware §5.1) hat einen L0-Test. Jeder Fehlercode (Spec 1 §9.5) hat einen Test in L1 oder L2. Jedes Abnahmekriterium unten hat eine Test-ID `M<n>-<nn>`.
Tool `tools/run_acceptance.py <Meilenstein>` führt die automatisierbaren Kriterien aus und füllt das Testprotokoll.

---

## 6. Abnahmekriterien (Vorschlag Claude, von Hannes änderbar)

**Messung:** Wiederholungszahlen n und Schwellen gelten für den realen Roboter, in der Simulation gilt „alle Wiederholungen“ (n/n). Positionsfehler mit Maßband und Bodenmarkierungen.

### M1 Kartieren und Fahren von A nach B
| ID | Kriterium | Verfahren | Schwelle |
|---|---|---|---|
| M1-01 | Raum kartiert (≤ 50 m²), Schleifenschluss, Karte gespeichert und neu geladen | Joystick-Fahrt, 3 Wandabstände nachmessen | ≤ 15 min, Fehler ≤ max(3 %, 5 cm) |
| M1-02 | Globale Lokalisierung | 5 Startposen (≥ 1 m Abstand, verschiedene Richtungen) | ≥ 4/5 in ≤ 30 s auf ≤ 0,15 m / 10° |
| M1-03 | Fahrt A→B | 10 Läufe, Strecke ≥ 3 m | ≥ 9/10, Endfehler ≤ 0,10 m und ≤ 10°, keine Bumper-Auslösung |
| M1-04 | Statisches Hindernis | Karton im Weg, 5 Läufe | 5/5 Umfahrung ohne Kontakt |
| M1-05 | Person quert den Weg | 5 Läufe | 5/5 Halt, danach Weiterfahrt ≤ 1 s nach Freigabe, Wartezeit ≤ `person_wait_s` + 2 s |
| M1-06 | Hardware-Not-Aus | 10 Läufe | Motoren stehen ≤ 0,2 s, Neustart nur nach Quittierung 10/10 |
| M1-07 | Software-Not-Stopp (UI) | 10 Läufe | Stillstand ≤ 0,5 s |
| M1-08 | Watchdog | Bridge beenden, 10 Läufe | Stillstand ≤ 0,6 s; nach 5 s Zustand WAIT_ACK |
| M1-09 | Abgrund vorn und hinten | 10 Läufe je Sensor mit 0,15 m/s | 10/10 Halt ≥ 3 cm vor der Kante |
| M1-10 | Bumper | 5 Läufe je Zone | 5/5 Stopp ≤ 0,2 s, Kontaktpunkt ≤ 0,10 m genau |
| M1-11 | WLAN-Ausfall | Router während der Fahrt abschalten, 5 Läufe | 5/5 Ziel erreicht |
| M1-12 | UI | 30 min Sitzung | Pose-Aktualisierung ≤ 1 s, keine Trennung, Start einer Fahrt ≤ 2 s nach dem Tippen |
| M1-13 | Automatische Sperrzone | Abgrund-Ereignis, danach neue Fahrt | 3/3 Zone erscheint, Plan meidet sie |
| M1-14 | Kalibrierung | C1–C8 und C10 abgeschlossen und dokumentiert | vollständig |

### M2 Gitterwagen von A nach B
| ID | Kriterium | Verfahren | Schwelle |
|---|---|---|---|
| M2-01 | Einfahren und Heben | Start an der Anfahrpose mit zufälligem Versatz bis ±0,05 m und ±6° | ≥ 9/10 real (10/10 Sim), Wagen hängt an allen vier Stiften |
| M2-02 | Transport A→B mit Wagen | 10 Läufe | ≥ 9/10, Wagen sitzt in der Zielführung |
| M2-03 | Lokalisierung mit Wagen | σ_xy während des Transports | ≤ 0,10 m, nie E202, 10/10 |
| M2-04 | Hub | 20 Zyklen, plus Endschalter abgezogen | 20/20 innerhalb Zeitlimit, Fehlerfall E107 korrekt |
| M2-05 | Not-Aus bei angehobenem Wagen | 3 Läufe | Wagen bleibt ≥ 60 s oben |
| M2-06 | Belegung | nach jedem Transport; absichtlich falsche Aufträge | Datenbank stimmt 10/10, Fehlermeldungen E301/E302 5/5 |
| M2-07 | Enge Passage mit Wagen (0,42 m) | 5 Läufe | 5/5 |
| M2-08 | Abbruch mit Wagen | 3 Läufe | Wagen kommt zurück zur Quelle 3/3 |
| M2-09 | Neustart mit Wagen an Bord | 3 Läufe | Dialog „Cart on board?“ und Fortsetzung korrekt 3/3 |

### M3 Rest
| ID | Kriterium | Verfahren | Schwelle |
|---|---|---|---|
| M3-01 | Warteschlange | 3 Aufträge mit Prioritäten 2/5/9, 5 Läufe (Sim 20) | Reihenfolge 9, 5, 2, Abbrechen funktioniert |
| M3-02 | Docking | 10 Anfahrten aus verschiedenen Startposen | ≥ 9/10, Stromfluss ≤ 5 s, Ladeende erkannt |
| M3-03 | Niedriger Akku | Schwelle für den Test angehoben, 3 Läufe | aktueller Auftrag wird beendet, danach Dock 3/3 |
| M3-04 | Nach Auftrag zum Laden | 10 Aufträge | 10/10 |
| M3-05 | Gebäudestromausfall | Ladegerät und Router aus | Auftrag beendet, Dock, Wartezustand, bei Netz Laden ≤ 15 s: 3/3 |
| M3-06 | Pi-Neustart im Auftrag | Netzteil des Pi trennen | Auftrag bleibt, Quittierung, Fortsetzung 3/3 (Schritte PICKUP_NAV/DEST_NAV), Handhabungsschritte → NEEDS_OPERATOR |
| M3-07 | Glas und Spiegel | Touch-Mapping, danach Fahrten daran vorbei | Punkte dauerhaft, Umfahrung 3/3 je Fläche |
| M3-08 | Fehlerkatalog | alle E-Codes erzeugen | 100 %, Text in EN/DE/IT |
| M3-09 | Dauerlauf | 2 h, ≥ 10 Aufträge | ≥ 90 % ohne Eingriff |
| M3-10 | Dokumentation | Relazione tecnica, Testprotokolle, Anleitungen in 3 Sprachen | vollständig |

---

## 7. Testprotokoll-Vorlage (je Meilenstein, `docs/<sprache>/tests/`)
Kopf: Meilenstein, Datum, Firmware-/Software-Commit, Tester, Raum, Akkuspannung, Wetter/Boden. Je Test: ID, Aufbau, Wiederholungen n, Ergebnisse (1…n, ok/nok, Messwert), Mittelwert/Streuung, Bewertung gegen die Schwelle, Foto-/Videoverweis, Bemerkung/Fehlerreport, Unterschrift. Claude erzeugt die Protokolle als Vorlage und trägt automatisch erfasste Werte ein.

---

## 8. Demo-Skript (freie Vorführung, Entwurf)

Vorbereitung (eine Stunde vor Beginn): Akku ≥ 12,5 V, bekannte gute Karte aktiv, Datenbank-Backup geladen (`/var/lib/robotros/backup/`), Wagen auf S1/S2, Raum frei, Probefahrt, Plan B bereit (Video der Vorführung und die Simulation auf dem Surface).
Ablauf (ca. 8–10 min, frei umstellbar): (1) Surface zeigt Karte und Pose, kurze Erklärung. (2) Auftrag per Button „Cart S1 → S2“. (3) Während der Fahrt kreuzt eine Person → Roboter wartet und fährt weiter. (4) Hindernis stellen → Umfahrung. (5) Zweiter Auftrag mit höherer Priorität zeigt die Warteschlange. (6) Router ausschalten → Roboter arbeitet weiter und fährt zum Dock. (7) Not-Aus drücken, Quittierung am Surface zeigen. (8) Docking und Laden zeigen. (9) optional Glas-/Spiegelfläche per Touch-Mapping.
Da die Vorführung frei ist, gilt die **Robustheitscheckliste:** keine Aufträge, die das Layout verlassen; „Clear contact points“ vor Beginn; Sperrzonen geprüft; Akku-Schwellen nicht überschritten.

---

## 9. Fragen ans Team (noch nicht gestellt, Rücksprache vorgesehen [Nutzer])

| Nr | Frage | Warum | Vorschlag Verantwortlich |
|---|---|---|---|
| 1 | Genaues Datum und Format des Wettbewerbs, Dauer von Präsentation und Vorführung | Zeitplan und Feature-Freeze (Vorschlag: 4 Wochen vorher) | Hannes |
| 2 | Gibt es ein schriftliches Regelwerk oder Bewertungsraster? | Dokumentation gezielt ausrichten | Hannes |
| 3 | Wer liefert welche Dokumente (Relazione tecnica, LEAN, Datenblattsammlung, Präsentation)? | Verantwortlichkeiten | Vorschlag: Software Lorenzo, Elektrik/CAD Isacco, Produkt/Präsentation Chiara, Gesamt Hannes |
| 4 | Budgetgrenze für Nachkäufe | Priorisierung der Nachkaufliste | Hannes |
| 5 | CAD-Maße: Roboterbreite, Radgröße, Radabstand, Wagenmaße (Außenmaß, lichte Weite zwischen den Beinen, Lochbild, Höhe der Unterseite) | **REQ-MECH-05 und REQ-MECH-02 sind kritisch** | Isacco |
| 6 | Hubmechanik: Winde, selbsthemmend? Hubhöhe, Stiftdurchmesser, Lochdurchmesser | Engage/Release | Isacco |
| 7 | Ladestation: Ladegerät (13,8 V CV, ≤ 0,72 A), Plattenmaße | Docking | Isacco |
| 8 | Router-Modell, Reichweite im Raum, feste IP-Möglichkeit | Netzwerk | Hannes |
| 9 | Surface-Modell, Linux-Touch-Test | UI | Hannes |
| 10 | Bestellung zweiter Nano Every, 2 × 5-V-Wandler ≥ 3 A, microSD A2/SSD, Datenblätter (Nano Every, Pololu, LB07) | Hardwarebasis | Hannes |
| 11 | Raumgröße, Abstellplatz-Positionen, Treppe/Kanten-Lage | Karte, Sperrzonen | gemeinsam |

---

## 10. Auflösung der offenen Punkte aus `projektwissen.md` §9

| Gruppe | Offener Punkt | Auflösung |
|---|---|---|
| A | Datum, Dauer, Ablauf der Präsentation/Vorführung | Frage 1 (Abschnitt 9). Vorführung ist frei [Nutzer], Demo-Skript Abschnitt 8. |
| A | Gantt, Budgetgrenze | Fragen 3–4, Gantt-Upload steht aus. |
| A | Dokumentverantwortliche, Relazione tecnica/LEAN | Frage 3. |
| B | Surface-Modell, Dual-Boot, Kernel | Modell nicht nötig, nur Browser. Ubuntu allein, **linux-surface-Kernel und iptsd** (Spec 1 §15.2), Check-Skript. |
| C | Hubmotor, Selbsthemmung | **ROB-13260** [Nutzer]; REQ-MECH-03; Zeitlimit C12. |
| C | Wagengewicht, Chassismaße, Bumper-Mechanik | CAD-Parameter, REQ-MECH-05/08, Frage 5. |
| D | Raddurchmesser, Spurbreite, Chassis | Parameter `wheel_radius_m`, `track_width_m` (CAD, C2, C3). |
| D | Encoder-Auflösung | C1. |
| D | Lasttest | C10, Fallback-Kette Abschnitt 4. |
| E | Dauer von Bumper-Hindernissen | 120 s, nach 3 Treffern dauerhaft, Touch-Mapping sofort (Spec 1 §8.2). |
| E | Manuelle Sperrzonen am Surface | **Ja**, Polygon per Touch (Spec 1 §8.3, §13.3). |
| E | Relais-Logik/Reset | Pegel im Pin-Header, `relay_probe`, REQ-ELEC-01 (Spec Firmware §1.3, §11.3). |
| F | Plattenmaße, Ausrichtung, Ladegerät, INA219-Einbau | REQ-MECH-04, Docking-Abfolge (Spec 1 §10.3), REQ-ELEC-02/04. |
| F | Verhalten ohne Netzstrom | D-17, Zustand WAITING_MAINS. |
| F | LB07, Leistungsbudget, Sicherungen | REQ-ELEC-03/05, Frage 10. Budget: Logikschiene ca. 1 A (2 Nano, Sensoren, LED bei 25 %, Relais), Sicherheitsschiene Motoren/Hub ≥ 3 A. [Schätzung] |
| G | GitHub verbunden, Repo-Name | steht aus (Anleitung `anleitung_github.md`, Name `RobotROS`). |
| H | Nav2-Einstellungen | Spec 1 §7.2, Feintuning T. |
| K | Router, IP, DDS, Reichweite | Hostname `robotros`/avahi, DDS nur lokal (D-20), Frage 8. |
| L | Türen, Schwellen, Rampen | **v1: keine Schwellen > 5 mm, keine Rampen, ein Raum** (Annahme für die Planung, beim Test C10 3-mm-Kante). |
| L | Glas/Spiegel | Bumper-Touch-Mapping ab M3 [Nutzer, Runde 25]. |
| M | Abnahmekriterien je Meilenstein | Abschnitt 6. |
| P | Pin-Lösung | D-01: Basisvariante A. Entscheidung der Hardware bleibt beim Team. |
| P | Nachkaufliste | Frage 10 und `bauteile_auswertung.md` §5. |
| P | Datenblätter Nano Every, Pololu, LB07 | nicht blockierend (Spec Firmware F7). |

---

## 11. Offene externe Abhängigkeiten (nicht durch Software lösbar)

1. **GitHub-Repository** anlegen und verbinden (Hannes).
2. **Hardwarelieferung** (Motoren, Lidar, Sensoren, Akku, Relais) und **Nachkäufe** (zweiter Nano Every, Wandler, Ladegerät und Station, Hubmechanik, Räder, Wagen, Führungen, Sicherungen, Kabel, microSD/SSD).
3. **CAD-Maße** für REQ-MECH-05 und REQ-MECH-02.
4. **Teamrücksprache** (Abschnitt 9).
5. **Pi- und Surface-Setup** (Ubuntu, ROS 2 Jazzy, linux-surface), Anleitung wird als nächstes Dokument geschrieben.

## 12. Hinweis zu Annahmen und Prüfpunkten
Mit **[prüfen]** markierte Punkte (Costmap-Footprint zur Laufzeit ändern, WDT-API des megaAVR, Nano-Every-USB-Kennung, Cyclone-Teilnehmerindex, BT-Knoten-Ports in Jazzy, Relais-Polarität) werden beim ersten Aufbau mit Test und Protokolleintrag geklärt. Sie sind in der Planung berücksichtigt und ändern die Architektur nicht.
