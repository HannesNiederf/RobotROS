# Änderungsmitteilung ÄM-01: Rückwärtseinfahrt unter die Gitterbox und eingeschränktes Lidar-Sichtfeld

Stand: 18.11.2026 (Revision 2: D-32 präzisiert, Lokalisierung mit Wagen) · **Hat Vorrang** vor den Abschnitten in `softwarespezifikation.md` (Spec 1), `spez_firmware_protokoll.md` (Spec FW) und `spez_parameter_tests.md` (Spec P), die in Abschnitt 4 genannt sind. Alles andere dort gilt unverändert. Eine Firmwareänderung ist **nicht** nötig.

## 1. Auslöser und Antworten des Nutzers (18.11.2026, nach Runde 25)

**Neue Info [Nutzer]:** Sobald der Roboter **rückwärts** unter eine Gitterbox einfährt, und bis er sie abgestellt hat und **vorwärts** wieder herausfährt, kann der Lidar **nur etwas mehr als 180° nach vorne** auswerten. Der Rest ist von der Gitterbox verdeckt. **Der Lidar soll auch bei der Fahrt mit Gitterbox zum Lokalisieren dienen, ihm fehlt nur der hintere Bereich.** [Nutzer, Klarstellung]

| Frage | Antwort [Nutzer] | Folge |
|---|---|---|
| Wie erkennt der Roboter beim Rückwärtsfahren die Endlage, wenn hinten kein Bumper ist? | „Odometrie und Lidar nach vorne“ | Keine neue Hardware am Heck. Die Endlage kommt aus Lokalisierung (Sektor-Scan) plus Odometrie, ergänzt durch Blockiererkennung der Räder (Software). |
| Wie fährt der Roboter in die Ladestation? | **Vorwärts.** Treffgenauigkeit nur auf Zentimeter nötig, die Pads für die Ladebürsten von unten sind groß genug. Hat das Laden nicht begonnen, obwohl die Position laut Lidar und Odometrie erreicht ist, soll der Roboter **etwas zurücksetzen und neu versuchen**. | Dock-Ablauf neu (D-34). |
| Wie ist die Gitterbox aufgebaut? | „Entweder drei Seiten Gitter bis zum Boden und eine Seite offen (auf festen Füßen), oder nur vier feste Füße“ | **Beide Varianten werden unterstützt** (`cart_occlusion_mode`: `SECTOR` oder `BOX`). Die Box rollt nicht. Das Team muss festlegen, welche gebaut wird. |
| Platz vor den Abstellplätzen? | „Mehr als 1 m frei“ | Wende-Manöver mit Wagen vor der Station ist möglich (REQ-ENV-01). |

## 2. Neue Entscheidungen

| ID | Entscheidung |
|---|---|
| **D-29** | **Rückwärtseinfahrt als Standard bei Abholen und Abstellen.** Der Roboter fährt mit dem Heck zuerst in die Box, die Front zeigt zur Öffnung. Danach fährt er vorwärts heraus. Nav2 fährt weiterhin nie rückwärts (`allow_reversing: false`). Rückwärtsfahren gibt es nur in den Manövern EngageCart und ReleaseCart, in der Recovery (0,05 m/s) und manuell per Joystick. |
| **D-30** | **Sichtfeld-Maske `cart_occlusion_mode`** ersetzt die Boxmaske aus Spec 1 §8.1. `SECTOR` (Standard, Box mit drei Wänden): nur der Winkelbereich ±`lidar_sector_half_angle_deg` (Startwert 95°, also 190°) um die Roboterfront bleibt gültig, alle anderen Strahlen werden maskiert. `BOX` (nur Füße): Boxmaske wie in Spec 1 §8.1. `NONE`: keine Maske. Die Maske ist aktiv **vom Beginn der Rückwärtsfahrt in die Box bis zum Beginn der Vorwärtsausfahrt nach dem Absetzen**. Gesteuert wird sie über das Topic `/mission/scan_mask` (`std_msgs/Bool`, TransientLocal), **nicht** mehr über den Hub-Endschalter. |
| **D-31** | **Maskierte Strahlen werden `+inf` gesetzt, nie `NaN`.** AMCL überspringt Strahlen ≥ `range_max`, bei `NaN` würden die Gewichte ungültig. Die Hindernis-Layer der Costmaps bekommen `inf_is_valid: false`, damit maskierte Strahlen keine Zellen löschen oder markieren. **[prüfen]** beim ersten Test, auch beim Collision Monitor. |
| **D-32** | **Lokalisierung mit Maske (Revision 2):** Der Lidar lokalisiert **auch mit Wagen weiter**. AMCL läuft normal mit dem Sektor-Scan (Verfolgung der Pose, wie ohne Wagen, nur mit ca. 190° statt 360° Sicht, die Odometrie überbrückt den Rest). Nur die **Wiederfindung nach Verlust** ist mit Wagen eingeschränkt, weil die Suchdrehung auf der Stelle mit blindem Heck nicht zulässig ist. Ablauf bei Verlust (σ_xy > 0,30 m länger als 5 s, gelockert wegen des kleineren Sichtfelds): (1) Roboter stoppt; (2) **lokale Neuinitialisierung** von AMCL um die letzte gute Pose (σ_xy 0,5 m, σ_yaw 0,5 rad, `relocalize_local_sigma_*`), stehend, bis zu 5 s; (3) bei fehlender Konvergenz **globale Neuinitialisierung stehend** (ohne Drehen), das Ergebnis wird nur akzeptiert, wenn die Pose konvergiert **und** höchstens `relocalize_max_jump_m` (1,5 m) von der letzten guten Pose entfernt liegt; (4) sonst `HELD_WITH_CART` mit Alert „Tap pose“ (Bediener tippt die Pose an, Spec 1 §7.3 Schritt 4). Ohne Wagen und beim Einschalten bleibt Spec 1 §7.3 unverändert (globale Lokalisierung, Suchdrehung). Nach Ende der Maske werden beide Costmaps im Bereich des Hindernis-Layers gelöscht (`clear_entirely`), weil hinten nichts gelöscht werden konnte. |
| **D-33** | **Neue Stationsgeometrie** (gilt für `STORAGE`): Die eingelernte Pose `S` ist die Roboterposition mit Wagen, die **Front zeigt aus der Box** (`û` = Blickrichtung). Dadurch ist die Richtung automatisch richtig, wenn der Roboter beim Einlernen unter dem Wagen steht. `P_pre = S + turn_distance·û`, Blickrichtung **zur Station** (yaw_S + π). `P_a = S + approach_distance·û`, Blickrichtung wie `û`. Nav2 fährt die Station über `P_pre` an. Für `DOCK` bleibt die alte Regel: Anfahrpose **hinter** der Dockpose, Fahrt vorwärts. |
| **D-34** | **Dock vorwärts mit Wiederholung** (ersetzt Spec 1 §10.3 Schritte 2–3), Ablauf in Abschnitt 3.3. |
| **D-35** | **Kein Hecksensor.** Die Endlage beim Rückwärtseinfahren bestimmt eine Regelung über die AMCL-Pose und die Odometrie, mit Abbruch durch Blockiererkennung. Der hintere Ultraschall bleibt Abgrundschutz (Nano) und meldet Hindernisse im Nahbereich, die nur im freien Abschnitt R1 zum Abbruch führen. |
| **D-36** | **Drehen auf der Stelle mit Wagen:** erlaubt (a) beim Wende-Manöver an `P_pre` und (b) bei Nav2-Drehungen mit höchstens 0,3 rad/s. Hinten ist der Roboter blind, deshalb gilt `REQ-ENV-01`, vor jeder Drehung zwei kurze Töne und das Orange-Lauflicht (Abschnitt 5). |
| **D-37** | **Slot-Check** am Vorprüfpunkt `P_pre` mit dem Front-Lidar: Beim **Abstellen** muss der Platz frei sein (höchstens `slot_free_max_points` Scanpunkte im Platz-Rechteck), sonst E302. Beim **Abholen** wird mit `cart_check_enabled` geprüft, dass ein Wagen im Platz steht (mindestens `cart_present_min_points` Punkte der Rückwand/Beine), sonst E301 mit der Option „Station als leer markieren“. Ab jetzt Standard `true` bei `SECTOR`, `false` bei `BOX`. |
| **D-38** | **Feinjustage beim Einlernen:** In der UI (Seite Stations) gibt es „Nudge“-Tasten: 1 cm vor und zurück, ±1° drehen (Service `/docking/nudge`, Geschwindigkeit 0,02 m/s). Damit lässt sich die Station zentimetergenau einlernen. |

## 3. Neue Abläufe (ersetzen Spec 1 §10.1 und §10.2, §10.3)

### 3.1 `EngageCart(station_id)` (Abholen)
Unterphasen, die im `MissionStatus.step` angezeigt werden: `TURN_PRE`, `SLOT_CHECK`, `TURN_AROUND`, `REVERSE_IN`, `LIFT`, `EXIT_FWD`.
1. **Navigation** zu `P_pre` (Toleranz 0,05 m, 0,10 rad), Roboter schaut zur Station.
2. **SLOT_CHECK** (D-37) mit voller 360°-Sicht (noch kein Wagen am Roboter).
3. **TURN_AROUND:** 2 kurze Töne (Buzzer 2), Lauflicht Orange auf allen Streifen, Drehung um 180° auf der Stelle mit `turn_speed` (0,3 rad/s). Drehrichtung: die Seite, in der die lokale Costmap im Radius `station_clear_radius` (0,5 m) weniger Kosten zeigt, sonst gegen den Uhrzeigersinn. Ende bei Gierwinkel-Fehler ≤ 0,05 rad. Ab hier schaut der Roboter **von der Station weg**.
4. **REVERSE_IN:** **Maske an** (Topic `/mission/scan_mask = true`). Zwei Abschnitte:
   - **R1** (frei): `turn_distance − approach_distance` (0,30 m) mit `reverse_fast_speed` (0,10 m/s). Abbruch bei hinterem Ultraschall `OBSTACLE` oder Abgrund (der Nano bremst bei Abgrund ohnehin).
   - **R2** (Führung): `approach_distance` (0,40 m) mit `reverse_slow_speed` (0,05 m/s). Regler auf der Station-Achse: Seitenabweichung `e_y` und Winkelfehler `e_θ` aus der AMCL-Pose, `ω = k_θ·e_θ + k_y·e_y` (Vorzeichen für Rückwärtsfahrt), `|ω| ≤ 0,3 rad/s`, Startwerte `k_θ = 2,0`, `k_y = 1,5` [T, C15]. Bei Ausfall der AMCL-Pose (σ zu groß) Weiterfahrt nur nach Odometrie bis zur Solllänge.
   - **Stoppkriterien** (das erste zutreffende): (a) Abstand zu `S` entlang der Achse ≤ `engage_stop_tol` (0,01 m); (b) Odometriestrecke ≥ `approach_distance + engage_hard_limit_extra` (0,03 m); (c) **Blockiererkennung**: Sollgeschwindigkeit ≠ 0, aber gemessene Radgeschwindigkeit < `stall_tps` (5 Ticks/s) für `stall_time_s` (0,5 s). Bei (c) gilt es als Anschlag, wenn der Restabstand ≤ 0,04 m ist, sonst E305.
5. **LIFT:** `$HUB` hoch bis `limit_top` oder `hub_timeout_s`, Verhalten bei Zeitüberschreitung wie Spec 1 §10.1 Punkt 5. Erfolg: `lifted = true`, Footprint und Profil CART, Wagen `CARRIED`.
6. **EXIT_FWD:** vorwärts `approach_distance` (0,40 m) mit 0,08 m/s bis `P_a`, Blinker an. Maske bleibt an (Wagen am Roboter). Danach übernimmt Nav2 am Punkt `P_a`.

### 3.2 `ReleaseCart(station_id)` (Abstellen)
1. Navigation zu `P_pre` des Zielplatzes (Wagen am Roboter, Maske an, Lokalisierung läuft mit dem Sektor-Scan weiter).
2. **SLOT_CHECK:** Platz muss frei sein (D-37), sonst E302 (`HELD_WITH_CART`).
3. **TURN_AROUND** wie 3.1 mit `turn_speed_cart` (0,25 rad/s).
4. **REVERSE_IN** wie 3.1 (Maske bereits an).
5. **LOWER:** `$HUB` runter bis `limit_bottom`. Erfolg: Wagen `STORED` an dieser Station, `lifted = false`.
6. **Maske aus**, beide Costmaps im Hindernis-Layer löschen (D-32), danach **EXIT_FWD** vorwärts bis `P_a`. Footprint zurück auf Roboter, Profil NORMAL. Hubfehler: Zustand `HELD_WITH_CART`, E306.

### 3.3 `DockCharger(station_id)` (vorwärts, D-34)
1. Navigation zur Anfahrpose `A_d` (`approach_distance` vor der Dockpose, Blick zum Dock, Toleranz 0,05 m und 0,10 rad).
2. **Fahrt vorwärts** mit 0,05 m/s. Stoppkriterien: (a) Abstand zur Dockpose ≤ `dock_tolerance_m` (0,02 m, nach AMCL und Odometrie); (b) **vorderer Bumper** (zählt als Anschlag, der Roboter bleibt stehen); (c) `v_charger ≥ charger_present_v`, dann noch 0,02 m weiter.
3. Innerhalb von 3 s `v_charger` prüfen, dann Laderelais an, innerhalb von 5 s muss `i_charge ≥ charge_detect_ma` fließen.
4. **Hat das Laden nicht begonnen** (keine Ladespannung oder kein Strom): **`dock_retry_backoff_m` (0,08 m) rückwärts** mit 0,05 m/s und erneut vorwärts nach Schritt 2, bis zu `dock_retries` (3) Versuchen.
5. Nach dem dritten Versuch ohne Ladespannung: Zustand `WAITING_MAINS` (Annahme: kein Netzstrom, Spec 1 D-17). Roboter bleibt am Dock, prüft alle 10 s die Spannung und wiederholt **alle 300 s** (`dock_reattempt_period_s`) einmal den Zyklus aus Schritt 4. Gab es Ladespannung, aber keinen Strom, bleibt es bei E111 beziehungsweise nach drei Versuchen bei E311.
`Undock` bleibt rückwärts 0,25 m (Roboter hat keinen Wagen, volles Sichtfeld).

## 4. Betroffene Abschnitte

| Dokument, Abschnitt | Änderung |
|---|---|
| Spec 1 §3.3 | zusätzliches Topic `/mission/scan_mask`; sonst unverändert |
| Spec 1 §7.2 (Collision Monitor, Costmaps) | Hindernis-Layer `inf_is_valid: false`; mit Maske sieht der Collision Monitor nur ±95° voraus |
| Spec 1 §7.3 | **Mit Wagen/Maske:** keine Suchdrehung, Wiederfindung nach Verlust gemäß D-32 (lokale Neuinitialisierung, dann globale stehend mit Plausibilitätsprüfung, dann Pose antippen). Ohne Wagen unverändert. |
| Spec 1 §8.1 | ersetzt durch D-30 und D-31 |
| Spec 1 §8.4 | Einlernen mit Front zur Öffnung, neue Anfahrpose, Nudge (D-33, D-38). Anfahrpose für `STORAGE` ist `P_pre`, für `DOCK` `A_d`. |
| Spec 1 §9.2, §9.3 | Phasen aus 3.1/3.2 im Status; Schrittnamen der Datenbank unverändert (`ENGAGE`, `RELEASE`) |
| Spec 1 §10.1, §10.2, §10.3 | ersetzt durch Abschnitt 3 |
| Spec 1 §6.4 | Recovery „kurz zurück“ mit Wagen nur 0,05 m und Buzzer, ohne Wagen unverändert |
| Spec 1 §14 (Simulation) | zwei Wagenmodelle: `walled` (drei Wände als dünne Boxen **mit** Visual, der Lidar sieht sie) und `feet`; Welt-Parameter `cart_type`; Station mit `û` nach der neuen Konvention |
| Spec 1 §18 | REQ-MECH-01: Längsreserve ±15 mm bleibt, Querreserve ±10 mm, Winkel ±5°. **REQ-MECH-02 ersetzt:** bei `SECTOR` muss die Öffnung ≥ 180° Sicht lassen (Messung C14) und die Gitterwände dürfen nicht in den Sektor ragen, bei `BOX` gilt der alte Text. REQ-MECH-04 unverändert (Pads groß genug). **Neu: REQ-ENV-01:** vor jeder Abstellplatz-Öffnung mindestens 1,0 m freie Fläche (Kreis mit 0,5 m Radius um `P_pre` frei). |
| Spec FW | keine Änderung. Blockiererkennung läuft auf dem Pi (Radgeschwindigkeit aus `/joint_states`). Signalplan: Zeile „Wende-Manöver“ = alle Streifen Lauflicht Orange, Buzzer vorher 2 × kurz. |
| Spec P §2.3, neue Parameter | siehe Abschnitt 5 |
| Spec P §3 | neu **C14** (Sichtsektor messen) und **C15** (Regler der Rückwärtseinfahrt) |
| Spec P §6 | geänderte und neue Kriterien in Abschnitt 6 |

## 5. Neue und geänderte Parameter (`mission.yaml`, `obstacles.yaml`)

| Parameter | Einheit | Startwert | Quelle |
|---|---|---|---|
| `cart_occlusion_mode` | – | `SECTOR` (`BOX`, `NONE`) | E / Team (Wagentyp) |
| `lidar_sector_half_angle_deg` | ° | 95 | K (C14) |
| `turn_distance` | m | 0,70 | T |
| `approach_distance` (je Station, Datenbank) | m | 0,40 | CAD, T |
| `reverse_fast_speed`, `reverse_slow_speed` | m/s | 0,10 / 0,05 | E |
| `exit_fwd_speed` | m/s | 0,08 | E |
| `turn_speed`, `turn_speed_cart` | rad/s | 0,30 / 0,25 | E |
| `station_clear_radius` | m | 0,5 | E |
| `engage_stop_tol` | m | 0,01 | T |
| `engage_hard_limit_extra` | m | 0,03 | T |
| `stall_tps`, `stall_time_s` | Ticks/s, s | 5 / 0,5 | T |
| `reverse_k_theta`, `reverse_k_y` | – | 2,0 / 1,5 | K (C15) |
| `slot_margin_m` | m | 0,04 | T |
| `slot_free_max_points` | – | 3 | T |
| `cart_present_min_points` | – | 8 | T |
| `cart_check_enabled` | bool | `true` bei `SECTOR`, sonst `false` | E |
| `rear_obstacle_abort_zone_m` | m | 0,30 (nur R1) | T |
| `dock_tolerance_m` | m | 0,02 | T |
| `dock_retry_backoff_m` | m | 0,08 | T |
| `dock_retries` | – | 3 | E |
| `dock_reattempt_period_s` | s | 300 | E |
| `loc_lost_sigma` (mit Maske) | m | 0,30 | T |
| `relocalize_local_sigma_xy`, `_yaw` | m, rad | 0,5 / 0,5 | T |
| `relocalize_max_jump_m` | m | 1,5 | T |
| `nudge_speed` | m/s | 0,02 | E |

## 6. Geänderte und neue Abnahmekriterien

| ID | Kriterium | Verfahren | Schwelle |
|---|---|---|---|
| M2-01 (neu gefasst) | Wende, Rückwärtseinfahren, Heben | Start an `P_pre` mit Versatz bis ±0,05 m und ±6° | ≥ 9/10 real (10/10 Sim), Wagen hängt an allen vier Stiften |
| M2-03 (geändert) | Lokalisierung mit Wagen und Sektormaske | σ_xy während Transport und Einfahren | ≤ 0,15 m, nie E202, 10/10 |
| M2-10 (neu) | Slot-Check | belegter Platz beim Abstellen und leerer Platz beim Abholen | 5/5 richtig erkannt (E302 / E301) |
| M2-11 (neu) | Endlage Rückwärtseinfahrt | Messung nach dem Stopp | längs ≤ 0,02 m, quer ≤ 0,015 m, ≥ 9/10 |
| M2-12 (neu) | Wende mit Wagen | 5 Läufe, Hindernis in 0,6 m Abstand | 5/5 ohne Kontakt, Töne und Licht vorher |
| M2-13 (neu) | Wiederfinden mit Wagen | Pose bei der Fahrt mit Wagen absichtlich um 0,3 bis 0,5 m verschieben (Initialpose setzen) | ≥ 4/5 findet die Lokalisierung selbst zurück (lokale Neuinitialisierung), sonst Pose-antippen-Dialog |
| M3-02 (erweitert) | Docking mit Wiederholung | Kontakte absichtlich abgeklebt | Zurücksetzen und Neuversuch laufen, nach 3 Versuchen E311 bzw. `WAITING_MAINS`, danach nach Entfernen des Klebebands Laden ≤ 5 min |
| M3-05 (erweitert) | Netzausfall | wie bisher plus Wiederholzyklus alle 300 s | Laden beginnt nach Netzrückkehr ≤ 5 min |

Neue Kalibrierverfahren:
- **C14 Sichtsektor:** Roboter mit dem echten Wagen in Endlage, `tools/lidar_sector.py` zeichnet 100 Scans auf und ermittelt die Winkel mit Rückgabewerten, die weiter als der Wagenrand liegen. `lidar_sector_half_angle_deg` = der kleinere Rand minus 3° Sicherheit. Ist der Sektor kleiner als 160°, `cart_occlusion_mode` und Mechanik mit dem Team prüfen.
- **C15 Regler Rückwärtseinfahrt:** 10 Einfahrten bei absichtlichem Anfangsversatz (±0,05 m, ±6°), Reglerverstärkungen anpassen, bis M2-11 erfüllt ist.

## 7. Risiko Lokalisierung mit halbem Sichtfeld
Mit ca. 190° Sicht ist die Pose in einem Raum bis 50 m² mit Wänden vor dem Roboter normalerweise gut bestimmbar. Schwach ist sie, wenn der Roboter entlang einer langen glatten Wand blickt, weil dann die Position längs der Wand nur die Odometrie liefert. Gegenmaßnahmen: langsame Fahrt mit Wagen (0,12 m/s), Prüfung in der Simulation und in M2-03, im Zweifel Wiederfinden nach D-32 und als letzte Stufe Pose antippen.

## 8. Offene Teamentscheidung
**Welcher Wagentyp wird gebaut?** `SECTOR` (drei Wände, eine offene Seite) oder `BOX` (nur vier Füße). Die Software unterstützt beide, die Standardannahme ist `SECTOR`. Bei `BOX` entfallen die Sichtfeld-Einschränkung und die Slot-Prüfung per Rückwand, der Lidar sieht dann nur vier Beine. Frage 12 in der Liste der Teamfragen (`spez_parameter_tests.md` §9).
