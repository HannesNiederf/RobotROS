# Softwarespezifikation „robotROS“ v1.0 (Entscheidungsdokument)

Stand: 18.11.2026 · Status: **verbindliche Entwicklungsgrundlage** (Hannes kann jede Entscheidung ändern; Änderungen als Eintrag im Abschnitt 19 festhalten)
Gehört zu: `claude/projektwissen.md` (Wissensbasis, Stand Runde 24), `claude/pinplan_nano.md`, `claude/bauteile_auswertung.md`, `claude/anleitung_github.md`
Weitere Teile der Spezifikation: `claude/spez_firmware_protokoll.md` (Nano-Firmware und serielles Protokoll), `claude/spez_parameter_tests.md` (Parameterkatalog, Kalibrierung, Tests, Abnahmekriterien, Auflösung aller offenen Punkte)

Legende: **[Nutzer]** vom Nutzer entschieden · **[Claude]** von Claude entschieden (Auftrag des Nutzers in Runde 25) · **[Wissen]** allgemeines Wissen · **[geprüft]** per Websuche bestätigt · **[prüfen]** beim ersten Aufbau zu verifizieren · **REQ-…** Anforderung an Mechanik/Elektrik, die die Software voraussetzt · **D-nn** Entscheidungsnummer
Regel bei Konflikten: Diese Spezifikation geht `projektwissen.md` vor, soweit sie später entstanden ist. Sprache: Bezeichner, Code, Kommentare und UI-Schlüssel sind Englisch, Fließtext der Projektdokumente Deutsch.

---

## 1. Runde 25 und Auftrag

### 1.1 Antworten des Nutzers (Runde 25, 18.11.2026)
| Frage | Antwort | Folge |
|---|---|---|
| Ablauf der Vorführung | „Freie Vorführung“ | Kein fester Parcours. Der Roboter muss in jeder Konstellation des Raums zuverlässig sein. Demo-Skript siehe `spez_parameter_tests.md` §8. |
| Hubmotor | „Einer der Backup-Motoren (ROB-13260)“ | Hubmotor = SparkFun ROB-13260 (ein Motor des Backup-Sets, der zweite bleibt Ersatz). Daten (Spannung, Drehmoment, Getriebe) fehlen noch → Hub-Zeitlimit als Kalibrierwert (C12). |
| Boden jetzt und später | „Vorerst glatter Raum, aber später, spätestens ab Meilenstein 3, müssen auch Glaswände oder Spiegel erfassbar (Bumper) sein“ | Kontaktpunkt-Ebene mit „Touch-Mapping“ ab M3 (Abschnitt 8.2). Lidar sieht Glas/Spiegel nicht verlässlich, der Bumper schon. |
| Rückfragen an das Team | „Ja, aber erst nach Rücksprache mit dem Team“ | Die Zuordnung zur ursprünglichen Frage ist in meiner Zusammenfassung nicht erhalten [Annahme: betrifft die Liste der Teamfragen zu Wettbewerbsdatum, Dokumentverantwortlichen und Budget]. Es wurde nichts ans Team verschickt. Die Teamfragen stehen in `spez_parameter_tests.md` §9. |

### 1.2 Auftrag zur Vollständigkeit
Auf die Frage „Gibt es noch etwas, das ich wissen sollte und bisher nicht gefragt habe?“ antwortete Hannes: **„Entscheide du alles so genau, dass du die Software ohne relevante Annahmen entwickeln kannst.“** [Nutzer]
Konsequenz: Alles, was früher als „offen“ oder „[Annahme]“ markiert war und die Software betrifft, ist hier entweder **entschieden** (D-nn), ein **Parameter** mit Startwert und Kalibrierverfahren (`spez_parameter_tests.md`) oder eine **Anforderung an Mechanik/Elektrik** (REQ-…, Abschnitt 18). Das frühere Gebot „keine Standardwerte ohne Rückfrage“ (Runde 14) ist durch diese Delegation für Softwaredetails aufgehoben. Hardwareentscheidungen (Teile kaufen, Schaltplan) bleiben beim Team.

---

## 2. Entscheidungsregister

| ID | Entscheidung | Begründung |
|---|---|---|
| D-01 | **Entwicklungs-Basisvariante für die Pins: Variante A (2× Nano Every, Rollen DRIVE und AUX).** Die Software kennt nur Rollen. Ein Wechsel auf Expander (B) oder Schieberegister (C) ändert nur Pin-Header und Geräteliste. | Hardware-Entscheidung bleibt beim Team, die Software ist davon entkoppelt. Zweiter Nano Every muss bestellt werden. |
| D-02 | ROS 2 Jazzy, Python (rclpy) für alle Knoten, C++ nur bei nachgewiesenem Leistungsproblem. Firmware C/C++ (PlatformIO). | [Nutzer: Claude schlägt vor] |
| D-03 | **Odometrie wird in der Bridge auf dem Pi berechnet** (Nano liefert nur Ticks). Kein ros2_control in v1. | Geometrie nur in YAML, Firmware bleibt stabil. |
| D-04 | **Sicherheitszustandsautomat liegt im Nano** (vier Zustände), der Pi spiegelt ihn nur. | Muss ohne Pi funktionieren. |
| D-05 | **Nach jedem Boot ist der Roboter gesperrt (LOCKED)**, bis am Surface quittiert wurde. Quittierung und Fortsetzen eines gespeicherten Auftrags sind ein gemeinsamer Dialog. | Konsistent mit Runde 16 und Runde 23. |
| D-06 | Bumper und Abgrund: **Nano sperrt nur die betroffene Richtung** (vorwärts bzw. rückwärts), der Pi führt die Wiederanfahr-Strategie aus (kurz zurück 0,10 m, Neuplanung). Ultraschall-Klasse „Hindernis“ blockiert nicht, nur „Abgrund“. | Docking unter dem Wagen darf nicht durch Führungen oder Beine blockiert werden. |
| D-07 | Sperrzonen: **manuell (Polygon am Surface) und automatisch (Abgrund-Treffer)**, beide dauerhaft in der Datenbank, am Surface löschbar. | Runde 23. |
| D-08 | **Kontakt-Ebene** als eigener, nie durch Lidar-Raytracing löschbarer Costmap-Layer (StaticLayer mit `use_maximum`). Bumper-Punkte temporär 120 s, nach 3 Treffern dauerhaft, im Touch-Mapping-Modus sofort dauerhaft. | Löst Glas/Spiegel und Runde 18/23. |
| D-09 | **Person im Weg:** keine Personenerkennung. Weg blockiert → Roboter steht, **wartet 4 s** (Parameter 3–5 s), plant neu, **3 Versuche**, danach Auftrag zurückgestellt (E204). | Runde 23/24. Der Unterschied „Mensch/Hindernis“ ergibt sich allein aus der Zeit. |
| D-10 | Lokalisierung: AMCL mit globaler Initialisierung. Ablauf: erst stehend 5 s, dann Drehung auf der Stelle, dann Fallback „Pose am Surface antippen“. | Runde 16. |
| D-11 | **Moduswechsel Kartieren ↔ Betrieb per Neustart des systemd-Dienstes** (ca. 15 s). Lifecycle-Umschaltung bleibt optionale Optimierung. | Robust, einfach zu testen. |
| D-12 | Footprint und Geschwindigkeitsprofil wechseln mit dem Hub-Zustand; der Lidar-Scan wird bei angehobenem Wagen durch eine Box-Maske von den Wagenbeinen bereinigt. | Auffälligkeit 5. |
| D-13 | **Stationen werden eingelernt:** Roboter fährt in Wagenposition (zentriert unter einem Wagen bzw. auf der Ladeplatte) und speichert seine Pose. Die Anfahrpose liegt `approach_distance` (0,40 m) davor. | Unabhängig von der CAD-Geometrie. |
| D-14 | Aufträge: Priorität 0–9 (9 = höchste, Standard 5), FIFO innerhalb der Priorität, **keine Unterbrechung** eines laufenden Auftrags, Abbrechen jederzeit. Prüfung beim Anlegen **strikt gegen aktuelle Belegung und Reservierungen**. | Runde 5, einfache Logik ohne Deadlock. |
| D-15 | Fehler **mit Wagen an Bord** → Zustand HELD_WITH_CART, Bediener entscheidet (Wiederholen, Wagen zurückbringen, anderen freien Platz wählen). Fehler ohne Wagen → Auftrag DEFERRED, Warteschlange läuft weiter. | Runde 8: „zurückstellen + Fehler mit Grund“. |
| D-16 | Laden: Ladeende per INA219-Strom; alle Schwellen als Platzhalter in YAML bis zum Akkutest (C9). | Runden 14, 21. |
| D-17 | **Kein Netzstrom an der Station:** Roboter bleibt an der Platte, prüft alle 10 s die Ladespannung, schließt das Laderelais erst bei Netz. Aufträge werden bei ausreichender Spannung trotzdem ausgeführt. | Runde 16 (Stromausfall). |
| D-18 | UI: Vanilla JavaScript (ES-Module, kein Build), roslib.js lokal, i18n-Dateien EN/DE/IT (EN Standard), Firefox-Kiosk auf dem Surface. | Runde 22. |
| D-19 | Simulation: Gazebo Harmonic, generische Welt 7 × 7 m. Zusätzlich ein **fake_nano** (Python), der das echte serielle Protokoll spricht. | Bridge und Mission lassen sich ohne Hardware testen. |
| D-20 | DDS: `rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42`, im Betrieb `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (kein DDS über WLAN, die UI nutzt nur rosbridge). **[geprüft]** In Jazzy ist `ROS_LOCALHOST_ONLY` wirkungslos. | Ein WLAN-Ausfall kann DDS nicht stören. |
| D-21 | Logik nutzt nie die Wanduhr, nur Monotonic-Zeit. chrony nur im Slew-Modus. | Pi hat ohne Netz keine verlässliche Uhrzeit. |
| D-22 | Keine Wagenerkennung in v1. Optional (M3, Standard aus): Prüfung der vier Wagenbeine im Lidar-Scan an der Anfahrpose. | Runde 14. |
| D-23 | Geschwindigkeitsprofile (Startwerte): normal 0,15 m/s · mit Wagen 0,12 m/s · Andocken/Einfahren 0,05 m/s · Drehen 0,6 rad/s (mit Wagen 0,5) · manuell 0,20 m/s (langsam 0,10). | Motoren unklar, nur Startwerte (C10). |
| D-24 | Die Bridge begrenzt auf 0,05 m/s, solange `geometry_calibrated: false`. | Erste Fahrt mit unbekannter Encoderauflösung ist ungefährlich. |
| D-25 | Fällt der Lasttest bei 5 V durch (C10), gilt die **Fallback-Kette** in `spez_parameter_tests.md` §4 (Motorschiene 6–7,2 V, `pwm_max`). | Runde 24: „Lösung finden“. |
| D-26 | Hub: nur Zeitlimit und Endschalter, keine Stromerkennung. Das Zeitlimit ist 1,5 × gemessene Hubzeit. | Kein Stromsensor im Hubpfad. |
| D-27 | Der Pi fährt bei kritischer Akkuspannung nicht selbsttätig herunter, solange eine Ladung möglich ist. Unterhalb `shutdown_v` stoppt er, alarmiert und fährt nach 60 s kontrolliert herunter. Ein Unterspannungsabschalter (Kanal REL_POWER) ist optional und für M3 vorgesehen. | Blei-Akku-Tiefentladung vermeiden. |
| D-28 | Alle Zustände, die einen Neustart überleben müssen, stehen in SQLite (Abschnitt 12). Nur der Mission-Knoten schreibt in die Datenbank. | Ein Schreiber, keine Locks. |

---

## 3. Systemarchitektur

### 3.1 Verteilung
- **Pi 5 (Ubuntu 24.04, ROS 2 Jazzy):** alles außer Anzeige. Läuft komplett ohne Surface und ohne WLAN. [Nutzer, Runde 15/16]
- **Surface (Ubuntu 24.04):** Firefox im Kiosk-Modus mit der Web-UI. Optional Entwicklungsrechner mit ROS 2, Gazebo und RViz2.
- **Nano(s):** Echtzeit, Sicherheit, Sensorik. Siehe `spez_firmware_protokoll.md`.

### 3.2 ROS-Pakete (Workspace `ros2_ws/src`)
| Paket | Inhalt |
|---|---|
| `robotros_interfaces` | Messages, Services, Actions (Abschnitt 5). Reines Schnittstellenpaket. |
| `robotros_description` | URDF/xacro (Roboter mit und ohne Wagen), Geometrie aus `robot.yaml`. |
| `robotros_bridge` | `nano_bridge` (serielle Bridge, Odometrie, TF), `fake_nano` (Python-Simulator des Nano), `protocol.py`, `kinematics.py`. |
| `robotros_map` | `map_manager`: Kartendateien, Moduswechsel, Stationen, Sperrzonen, Kontaktpunkte, Web-Kartenbild, Keepout-Maske, Kontakt-Karte. |
| `robotros_obstacles` | `scan_masker` (Selbst- und Wagenmaske), `contact_obstacles` (Bumper/Ultraschall → Punkte), `footprint_switcher`. |
| `robotros_mission` | `mission_manager` (Zustandsautomat, Auftragswarteschlange, Datenbank), `charging_manager`, Fehlercodes. |
| `robotros_docking` | `docking_server` mit den Actions EngageCart, ReleaseCart, DockCharger, Undock, Localize. |
| `robotros_bringup` | Launch-Dateien, alle YAML-Konfigurationen, Behavior-Tree-XML, systemd-Starter. |
| `robotros_sim` | Gazebo-Welten, Modelle, `sim_hw_adapter`, `sim_person`, Simulations-Launch und Integrationstests. |
| `robotros_web` | Startet den statischen Webserver (Port 8080) und rosbridge (Port 9090). |

Externe Pakete (apt, `ros-jazzy-…`): navigation2, nav2-bringup, slam-toolbox, rosbridge-suite, rplidar-ros, twist-mux, ros-gz, xacro, robot-state-publisher, rmw-cyclonedds-cpp, plus Python `pyserial`, `numpy`. [Wissen, beim Setup prüfen]

### 3.3 Datenfluss (vereinfacht)
```
Surface (Browser) ──WebSocket 9090──► rosbridge ─┬─► mission_manager ──► Nav2 (NavigateToPose, eigener BT)
        ▲ HTTP 8080 (statische UI)               ├─► map_manager ──► map_server / slam_toolbox / Keepout / Kontakt-Karte
                                                 ├─► docking_server ──► /cmd_vel_docking
 joystick ► /cmd_vel_manual ─► twist_mux A ─► collision_monitor ─► twist_mux B ─► /cmd_vel ─► nano_bridge ─USB─► Nano DRIVE (Motoren, Encoder, US, Bumper, Not-Aus, Arming)
 Nav2 ► /cmd_vel_nav ──┘                                         docking ─────────┘          └USB─► Nano AUX (Hub, LED, Buzzer, Relais, Spannung, INA219)
 rplidar ► /scan ► scan_masker ► /scan_filtered ► AMCL / SLAM / Costmaps / Collision Monitor
```
Zwei Multiplexer: A (Joystick vor Nav2) läuft **durch** den Collision Monitor, B (Docking vor allem anderen) läuft **hinter** ihm, weil die Wagenbeine beim Einfahren sonst als Hindernis stoppen würden. Sicherheit beim Docking liefert allein der Nano (Bumper/Abgrund). **[Claude]**

---

## 4. Frames und Geometrie

| Frame | Beziehung | Quelle |
|---|---|---|
| `map` → `odom` | Lokalisierung | AMCL (Betrieb) oder SLAM Toolbox (Kartieren) |
| `odom` → `base_footprint` | Odometrie | `nano_bridge` (50 Hz) |
| `base_footprint` → `base_link` | z-Versatz | `robot_state_publisher` |
| `base_link` → `laser` | Lidar-Montage (`lidar_x`, `lidar_z`, `lidar_yaw`) | URDF |
| `base_link` → `us_front`, `us_rear` | Ultraschall (x, z, Neigung `us_pitch_deg`) | URDF |
| `base_link` → `bumper` | vordere Kante (x = `bumper_front_x`) | URDF |

Geometrie-Startwerte (Platzhalter bis CAD, siehe REQ-MECH-05): Roboter 0,26 × 0,26 m, Radradius 0,0325 m, Spurbreite 0,16 m, Lidar bei x = 0, z = 0,12 m. Footprint ohne Wagen: Quadrat 0,26 m; mit Wagen: Quadrat 0,32 m (Wagen 0,30 m plus 1 cm je Seite).
Zeitstempel: Sensorzeit = Ankunftszeit minus `serial_latency_s` (0,005 s).

---

## 5. ROS-Schnittstellen

### 5.1 Messages (`robotros_interfaces/msg`)
| Message | Felder |
|---|---|
| `SafetyState` | `header`, `uint8 state` (0 BOOT_LOCKED, 1 ESTOP, 2 WAIT_ACK, 3 ARMED), `bool estop_pressed`, `bool relay_on`, `bool block_forward`, `bool block_backward`, `bool pi_link_lost`, `bool software_stop`, `uint16 flags` |
| `BumperState` | `header`, `bool left`, `bool middle`, `bool right` |
| `RangeState` | `header`, `uint16 front_mm`, `uint16 rear_mm` (65535 = kein Echo), `uint8 front_class`, `uint8 rear_class` (0 UNKNOWN, 1 FLOOR, 2 CLIFF, 3 OBSTACLE) |
| `PowerState` | `header`, `float32 v_batt`, `v_charger`, `v_rail`, `i_charge` (A, positiv = lädt), `bool charger_present`, `bool charge_relay_on`, `float32 soc_estimate` (−1 = unbekannt) |
| `HubState` | `header`, `uint8 state` (0 IDLE, 1 MOVING_UP, 2 MOVING_DOWN, 3 TOP, 4 BOTTOM, 5 FAULT), `bool limit_top`, `bool limit_bottom`, `uint8 fault` |
| `HubCommand` | `uint8 command` (0 STOP, 1 UP, 2 DOWN), `uint8 pwm` |
| `LedCommand` | `uint8 strip` (0 LEFT, 1 RIGHT, 2 REAR), `uint8 pattern` (0 OFF, 1 SOLID, 2 BLINK, 3 SWEEP), `uint8 r,g,b` |
| `BuzzerCommand` | `uint8 pattern` (0 OFF, 1 REVERSE, 2 SHORT, 3 ALARM) |
| `Station` | `uint32 id`, `string name`, `string kind` (STORAGE/DOCK), `geometry_msgs/Pose2D pose`, `float32 approach_distance`, `bool enabled`, `bool occupied`, `string cart_name` |
| `Order` | `uint32 id`, `string type` (TRANSPORT/GOTO_POINT/GOTO_DOCK), `uint8 priority`, `uint32 source_station_id`, `uint32 dest_station_id`, `Pose2D goal`, `string state`, `string step`, `string error_code`, `string error_detail`, `uint8 attempts`, `string created_at`, `string origin` (USER/SYSTEM) |
| `MissionStatus` | `header`, `string mode` (NORMAL/MAPPING/MANUAL), `string state` (Abschnitt 9.2), `uint32 active_order_id`, `string step`, `float32 progress`, `string message_key`, `string error_code`, `bool lifted`, `float32 loc_sigma_xy`, `string waiting_reason`, `uint32 queue_length` |
| `OperatorAlert` | `uint32 id`, `string severity` (INFO/WARN/ERROR), `string code`, `string detail`, `bool needs_action`, `string[] options` |
| `KeepoutZone` | `uint32 id`, `string source` (MANUAL/CLIFF_AUTO), `geometry_msgs/Polygon polygon`, `string note` |
| `ContactPoint` | `uint32 id`, `float32 x`, `float32 y`, `uint16 hits`, `bool permanent` |

Listen-Messages (`OrderList`, `StationList`, `KeepoutZoneList`, `ContactPointList`) enthalten nur ein Array der Einzelmessage.

### 5.2 Services und Actions
| Name | Typ | Zweck |
|---|---|---|
| `/hw/set_arm` | `SetArm` (`bool arm` → `success`, `state`) | Arming/Disarming des Sicherheitsrelais über den Nano |
| `/hw/set_relay` | `SetRelay` (`channel`, `on`) | Laderelais u. a. |
| `/mission/create_order` | `CreateOrder` (type, priority, source, dest, goal → success, order_id, error_code, detail) | Auftrag anlegen (Prüfregeln 9.3) |
| `/mission/order_action` | `OrderAction` (order_id, action = CANCEL/RETRY/SET_PRIORITY/RESUME_AT_STEP, value) | Auftrag verändern |
| `/mission/ack_safety` | `AckSafety` (`resume_mode` = RESUME/KEEP_PAUSED/CANCEL_ORDER → success, error_code) | Quittierung (Abschnitt 6.3) |
| `/mission/resolve_alert` | `ResolveAlert` (alert_id, option) | Entscheidung des Bedieners |
| `/mission/set_mode` | `SetMode` (NORMAL/MAPPING/MANUAL) | Betriebsart |
| `/mission/set_cart_location` | `SetCartLocation` (cart_name, station_id oder 0) | Belegung korrigieren |
| `/mission/software_estop` | `std_srvs/Trigger` | Software-Not-Stopp (Disarm) |
| `/map/save`, `/map/list`, `/map/activate` | `SaveMap` usw. | Kartenverwaltung |
| `/map/upsert_station`, `/map/delete_station` | `UpsertStation`, `DeleteStation` | Stationen (Einlernen siehe 8.4) |
| `/map/upsert_zone`, `/map/delete_zone` | `UpsertKeepoutZone`, `DeleteKeepoutZone` | Sperrzonen |
| `/map/delete_contacts` | `DeleteContactPoints` (ids oder Radius) | Kontaktpunkte löschen |
| `/map/set_touch_mapping` | `SetBool` | Touch-Mapping-Modus ein/aus |
| `/map/get_snapshot` | `GetSnapshot` | Liefert Kartenbild, Stationen, Zonen, Kontaktpunkte einmalig (UI-Start, Reconnect) |
| `/localize/*` | Action `Localize` | globale Lokalisierung (Abschnitt 7.3) |
| `/docking/engage_cart`, `/docking/release_cart`, `/docking/dock_charger`, `/docking/undock` | Actions | Abschnitt 10 |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigation, mit eigenem BT (7.4) |

### 5.3 Topics
| Topic | Typ | Rate | QoS | Von → An |
|---|---|---|---|---|
| `/scan` | `LaserScan` | 5,5–10 Hz | SensorData | rplidar → scan_masker |
| `/scan_filtered` | `LaserScan` | wie /scan | SensorData | scan_masker → AMCL, SLAM, Costmaps, Collision Monitor |
| `/odom` | `Odometry` | 50 Hz | SensorData | bridge → Nav2, AMCL |
| `/joint_states` | `JointState` | 20 Hz | SensorData | bridge |
| `/hw/safety` | `SafetyState` | 5 Hz + bei Änderung | Reliable | bridge → mission, UI |
| `/hw/bumper` | `BumperState` | bei Änderung + 10 Hz | Reliable | bridge → contact_obstacles, mission |
| `/hw/range` | `RangeState` | 16 Hz | SensorData | bridge → contact_obstacles, mission |
| `/hw/power` | `PowerState` | 2 Hz | Reliable | bridge → charging_manager, UI |
| `/hw/hub_state` | `HubState` | bei Änderung + 2 Hz | Reliable | bridge → mission, docking, scan_masker, footprint_switcher |
| `/hw/cmd/hub`, `/hw/cmd/led`, `/hw/cmd/buzzer` | Commands | bei Bedarf | Reliable | mission/docking → bridge |
| `/cmd_vel_manual` | `Twist` | 20 Hz | Reliable | UI → twist_mux A |
| `/cmd_vel_nav` | `Twist` | 20 Hz | Reliable | Nav2 (nach Velocity Smoother) → twist_mux A |
| `/cmd_vel_docking` | `Twist` | 20 Hz | Reliable | docking_server → twist_mux B |
| `/cmd_vel` | `Twist` | 20 Hz | Reliable | twist_mux B → bridge |
| `/footprint` | `PolygonStamped` | bei Änderung | TransientLocal | footprint_switcher → Collision Monitor |
| `/contact_map` | `OccupancyGrid` | bei Änderung | TransientLocal | map_manager → Kontakt-Layer in beiden Costmaps |
| `/keepout_mask`, `/costmap_filter_info` | `OccupancyGrid`, `CostmapFilterInfo` | bei Änderung | TransientLocal | map_manager → Keepout-Filter |
| `/mission/status` | `MissionStatus` | 2 Hz + bei Änderung | Reliable | mission → UI |
| `/mission/orders` | `OrderList` | bei Änderung + alle 5 s | Reliable | mission → UI |
| `/mission/alerts` | `OperatorAlert` | bei Ereignis | Reliable | mission → UI |
| `/web/map_image` | `CompressedImage` (PNG) | bei Änderung + alle 5 s | Reliable | map_manager → UI |
| `/web/map_meta` | `MapMeta` (Auflösung, Ursprung, Größe) | wie Kartenbild | Reliable | map_manager → UI |
| `/web/stations`, `/web/zones`, `/web/contacts` | Listen-Messages | bei Änderung + alle 5 s | Reliable | map_manager → UI |
| `/robot_pose` | `PoseStamped` (Frame `map`) | 5 Hz | Reliable | map_manager → UI |

**Wichtig:** Die Web-Pfade verlassen sich **nicht** auf „Latching“. Statische Daten werden zusätzlich alle 5 s erneut gesendet, und die UI holt beim Start oder Reconnect `/map/get_snapshot`. **[Claude, Absicherung gegen rosbridge-QoS-Unterschiede]**

---

## 6. Sicherheit

### 6.1 Zustandsautomat (im Nano)
| Zustand | Bedeutung | Relais | Übergang |
|---|---|---|---|
| `BOOT_LOCKED` | Nach Reset/Einschalten | aus | `$ARM,1` und Not-Aus gelöst → ARMED |
| `ESTOP` | Taster gedrückt | aus (hart durch Taster) | Taster gelöst → WAIT_ACK |
| `WAIT_ACK` | Taster gelöst oder Disarm per Software, wartet auf Quittierung | aus | `$ARM,1` → ARMED |
| `ARMED` | Betrieb | an | Taster → ESTOP; `$ARM,0` → WAIT_ACK; **5 s ohne gültigen Pi-Befehl → WAIT_ACK** |
Nach dem Wechsel in ARMED sind alle Sollwerte 0, erste Fahrbefehle werden nach 200 ms akzeptiert.

### 6.2 Reaktionen
| Ereignis | Nano | Pi |
|---|---|---|
| Not-Aus | Relais hart aus (Taster in der Spulenversorgung), Zustand ESTOP | Mission pausiert (`PAUSED_SAFETY`), Alert „Emergency stop“ |
| Software-Not-Stopp (UI) | `$ARM,0` → WAIT_ACK | wie oben, ohne Taster |
| Abgrund vorn | Vorwärts sperren, Ziel auf 0, Bremse | Event → Recovery 6.4, automatische Sperrzone (8.3) |
| Abgrund hinten | Rückwärts sperren, Ziel auf 0 | wie oben |
| Bumper links/mitte/rechts | Vorwärts sperren, Ziel auf 0 | Recovery 6.4, Kontaktpunkt (8.2) |
| Watchdog 0,5 s | Ziel 0, Bremse, Flag `pi_link_lost` | Bridge-Wiederverbindung |
| Watchdog 5 s | → WAIT_ACK | Neustart-Dialog mit Quittierung |
| Hub-Zeitüberschreitung | Hubmotor aus, `HUB_FAULT` | Fehler E107 (9.5) |

### 6.3 Quittierung am Surface
Dialog erscheint bei Zustand BOOT_LOCKED oder WAIT_ACK. Schaltfläche „Arm robot“ ist nur aktiv, wenn `estop_pressed == false`. Wenn ein aktiver Auftrag gespeichert ist, bietet der Dialog drei Schaltflächen: **Arm and resume**, **Arm and keep paused**, **Arm and cancel order** (`resume_mode`). Die Bridge sendet `$ARM,1` und meldet Erfolg erst, wenn der Nano ARMED zurückmeldet (Zeitlimit 1 s, sonst E102).

### 6.4 Recovery nach Bumper/Abgrund
1. Nano hat gestoppt. Mission setzt `waiting_reason = CONTACT`, bricht den aktuellen Nav2-Goal ab.
2. Wenn nicht rückwärts gesperrt: 0,10 m rückwärts mit 0,05 m/s (`recovery_backup_m`), Buzzer REVERSE.
3. Kontaktpunkt/Sperrzone in die Karte schreiben (Kontakt-Ebene, 8.2 bzw. 8.3), danach sofortige Neuplanung.
4. Dritter Kontakt am selben Auftrag → Auftrag DEFERRED mit E106 (Bumper) bzw. E105 (Abgrund). Zähler `contact_retries: 3`.

---

## 7. Navigation, Kartierung, Lokalisierung

### 7.1 Kartieren (Modus MAPPING)
- Start über UI. `map_manager` schreibt `/var/lib/robotros/mode.env` (`ROBOTROS_MODE=mapping`) und startet den Dienst neu. [D-11]
- Im Modus MAPPING laufen: Bridge, Lidar, `scan_masker`, `slam_toolbox` (`online_async`), rosbridge, UI-Server, `map_manager`, twist_mux. Nav2 und AMCL laufen **nicht**. Fahren nur per Joystick (`/cmd_vel_manual`, Collision Monitor aktiv, Abgrund/Bumper im Nano aktiv).
- SLAM-Startwerte: Auflösung 0,05 m, `max_laser_range` 8,0 m (Raum ≤ 50 m²), `minimum_travel_distance` 0,1 m, `minimum_travel_heading` 0,1 rad, `map_update_interval` 2 s, Schleifenschluss an, `minimum_time_interval` 0,3 s.
- Speichern: `/map/save` ruft den Map-Saver (PGM+YAML) **und** die Pose-Graph-Serialisierung von slam_toolbox auf. Dateien `/var/lib/robotros/maps/<name>.{pgm,yaml,posegraph,data}`. Eintrag in `maps`, wird aktive Karte.
- Nach dem Speichern: Moduswechsel zurück auf NORMAL. Neue Karte → bestehende Stationen und Zonen gehören zur alten `map_id` (UI warnt, Übernahme per „Copy to new map“ nicht in v1).

### 7.2 Betrieb (Modus NORMAL)
- `map_server` lädt die aktive Karte, `amcl` lokalisiert, Nav2 (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, `collision_monitor`, `lifecycle_manager`) läuft.
- **AMCL:** Eingabe `/scan_filtered`, `max_particles` 2000, `min_particles` 500, `laser_model_type: likelihood_field`, `update_min_d` 0,05, `update_min_a` 0,10, `set_initial_pose: false`, `odom_model_type: diff-corrected`, `alpha1..4` 0,2 (Startwerte wegen Schlupf ohne IMU).
- **Kostkarten:** Auflösung 0,05 m. Global: Layer `static_layer`, `contact_layer`, `obstacle_layer`, `inflation_layer`, Filter `keepout_filter`. Lokal (rollend 3 × 3 m): `contact_layer`, `obstacle_layer`, `inflation_layer`, Filter `keepout_filter`.
- `contact_layer`: `nav2_costmap_2d::StaticLayer` mit `map_topic: /contact_map`, `map_subscribe_transient_local: true`, **`use_maximum: true`** (sonst überschreibt der Layer die anderen). Lidar-Raytracing löscht diesen Layer nie. [D-08]
- `obstacle_layer`: Quelle `/scan_filtered`, `obstacle_max_range` 4,0 m, `raytrace_max_range` 5,0 m.
- **Inflation:** `inflation_radius` 0,30 m, `cost_scaling_factor` 5,0. Kleinste befahrbare Gasse = Footprintbreite + 0,10 m (0,36 m ohne, 0,42 m mit Wagen).
- **Planer:** `nav2_smac_planner/SmacPlanner2D`, `allow_unknown: false`, `tolerance: 0.05`, `max_planning_time: 2.0`, `cost_travel_multiplier: 2.0`.
- **Regler:** `nav2_regulated_pure_pursuit_controller`: `desired_linear_vel` aus dem Profil, `lookahead_dist` 0,40 (min 0,20, max 0,60), `use_rotate_to_heading: true`, `rotate_to_heading_angular_vel` 0,6, `allow_reversing: false`, `use_collision_detection: true`, `max_allowed_time_to_collision_up_to_carrot` 1,0, Kostenregelung an (`regulated_linear_scaling_min_speed` 0,05).
- **Zielprüfer:** `xy_goal_tolerance` 0,04 m, `yaw_goal_tolerance` 0,08 rad, `stateful: true` (liegt enger als das Abnahmekriterium 0,10 m / 10°).
- **Velocity Smoother:** `max_velocity [0.20, 0.0, 0.8]`, `min_velocity [-0.10, 0.0, -0.8]`, `max_accel [0.30, 0.0, 1.5]`, `max_decel [-0.50, 0.0, -2.0]`, `feedback: OPEN_LOOP`.
- **Collision Monitor:** Quelle `/scan_filtered`, Polygon vom Typ `approach` mit `time_before_collision` 1,0 s und dynamischem Footprint (`footprint_topic: footprint`), Aktion STOP. **[geprüft: Der Collision Monitor unterstützt dynamische Footprints; in Jazzy beim ersten Test verifizieren]**
- **Footprint-Wechsel:** `footprint_switcher` setzt bei angehobenem Wagen den Footprint der Costmaps (Parameter `footprint` per `set_parameters`, **[prüfen]** ob zur Laufzeit änderbar) und veröffentlicht `/footprint` für den Collision Monitor. Wenn die Laufzeitänderung der Costmap nicht funktioniert, Fallback: beide Costmap-Knoten per Lifecycle neu konfigurieren (Wechsel dauert ca. 2 s, nur an Hub-Stillstand).

### 7.3 Lokalisierung (Action `Localize`)
1. AMCL `reinitialize_global_localization` aufrufen.
2. **Stehend 5 s warten** (`loc_wait_s`). Konvergenz, wenn σ_xy ≤ 0,10 m und σ_yaw ≤ 0,15 rad (aus der AMCL-Kovarianz) über 2 s stabil.
3. Nicht konvergiert und beide Ultraschallklassen = FLOOR: Drehung auf der Stelle mit 0,3 rad/s, bis zu 720°, nach jeder 90°-Teildrehung Prüfung wie in Schritt 2.
4. Nicht konvergiert, oder ein Ultraschall meldet CLIFF: Alert „Localization failed – tap approximate position“. Der Bediener tippt die Pose (Position und Richtung per Ziehen) → Initialpose mit σ 0,5 m, danach Schritt 2 erneut.
5. Erfolg: `loc_sigma_xy` im Status, Zustand IDLE.
**Überwachung im Betrieb:** σ_xy > 0,25 m länger als 5 s → Navigation stoppt, E202, erneut Localize. [Claude]

### 7.4 Behavior Tree (`robotros_bringup/behavior_trees/navigate_wait_replan.xml`)
Aufbau: `RecoveryNode` (3 Wiederholungen) um `PipelineSequence`: `RateController` (1 Hz) → `ComputePathToPose`, danach `FollowPath`. Recovery-Zweig: `Wait` (Parameter `person_wait_s`, 4 s) → `ClearEntireCostmap` (lokal). Plan-Fehler: einmal `ClearEntireCostmap` (global) und neu planen. Nach der dritten Wiederholung: Aktion fehlgeschlagen → Mission setzt E204 (Weg blockiert) oder E203 (kein Pfad). **[Claude]** Knotennamen sind Nav2-Standard-BT-Knoten; Port-Syntax beim Anlegen der XML gegen die Jazzy-Version prüfen.
Ergebnis für den Nutzer: Mensch quert den Weg → Regler meldet drohende Kollision, Roboter steht, nach 4 s wird neu geplant; ist der Mensch weg, folgt der gleiche Weg, steht dort ein festes Hindernis, wählt der Planer einen Umweg. Der Collision Monitor stoppt zusätzlich sofort bei plötzlicher Annäherung.

### 7.5 Geschwindigkeitswahl
`speed_profile` (NORMAL, CART, DOCK, MANUAL, MANUAL_SLOW) im Mission-Knoten. Wechsel setzt Parameter des Reglers und des Velocity Smoothers per `set_parameters` (Werte aus `speed_profiles.yaml`). Bei angehobenem Wagen automatisch CART.

---

## 8. Hindernisse, Kontaktpunkte, Sperrzonen, Stationen

### 8.1 Lidar-Maskierung (`scan_masker`)
- **Selbstmaske:** Liste von Rechtecken im `base_link` (`self_mask_boxes`), z. B. Befestigungsstützen des Lidars. Punkte in der Maske → `inf`.
- **Wagenmaske:** Wenn `/hw/hub_state.limit_top == true` oder Zustand `lifted`, Rechteck `cart_mask_box` (±0,17 m + `cart_mask_margin` 0,03 m) → Punkte darin werden ausgeblendet (Wagenbeine). Gilt nur, wenn REQ-MECH-02 erfüllt ist (nur Stützen schneiden die Scanebene).
- Winkelbereich und Reichweite: 0,15–8,0 m (`range_min`, `range_max`).
- Implementierung: numpy, Rate = Scanrate.

### 8.2 Kontakt-Ebene (`contact_obstacles`)
- Eingabe: `/hw/bumper`, `/hw/range`, AMCL-Pose.
- **Bumper-Punkt:** Beim Anstieg eines Bumperbits wird ein Streifen an der Vorderkante des Roboters erzeugt: Zone links y = +`bumper_zone_y` (0,08 m), Mitte y = 0, rechts y = −0,08 m, jeweils Breite `contact_zone_width` 0,09 m, x = `bumper_front_x` + 0,02 m, in `map` umgerechnet. Mehrere gleichzeitige Zonen = mehrere Streifen.
- **Lebensdauer:** 120 s (`contact_lifetime_s`). Ein Punkt im Radius 0,15 m (`contact_merge_radius`) zählt als gleicher; **ab 3 Treffern** (`contact_promote_hits`) wird er **dauerhaft** (Tabelle `contact_points`). Im **Touch-Mapping-Modus** ist jeder Treffer sofort dauerhaft.
- **Ultraschall „Hindernis“** erzeugt Punkte mit 30 s Lebensdauer (nie dauerhaft). Abgrund siehe 8.3.
- Ausgabe: `/contact_map` (OccupancyGrid, Größe wie die Basiskarte, 100 = belegt, 0 = frei) bei Änderung und auf 1 Hz begrenzt, plus `/web/contacts`.
- Löschen: UI „Clear contact points“ (alle oder Radius), Einzelpunkt per Antippen.
- Ab M3: Touch-Mapping für Glaswände und Spiegel. Vorgehen: Modus einschalten, Roboter langsam (0,05 m/s) per Joystick gegen die Wand fahren, Bumper löst aus, Punkt wird dauerhaft; wiederholen entlang der Fläche im Abstand von ≤ 0,09 m. Die Kontaktpunkte gehören zur `map_id`.

### 8.3 Sperrzonen
- **Manuell:** UI zeichnet ein Polygon (3–8 Ecken per Antippen), `note` optional. Quelle MANUAL.
- **Automatisch (Abgrund):** Bei einem Abgrund-Ereignis wird ein Rechteck erzeugt: Breite = Footprintbreite + 0,10 m, Tiefe 0,20 m, Mitte am Bodenpunkt des Sensors (Sensorabstand `us_ground_x` entlang der Fahrtrichtung, Rückwärts analog), Quelle CLIFF_AUTO. Überlappende automatische Zonen werden vereinigt. Obergrenze `cliff_auto_zone_max` 30.
- **Keepout-Maske:** `map_manager` rastert alle Zonen der aktiven Karte in eine `OccupancyGrid` (100 = gesperrt) und veröffentlicht `/keepout_mask` und `/costmap_filter_info` (Typ 0 = Keepout) bei jeder Änderung.
- Ziel- und Anfahrposen in einer Sperrzone werden beim Anlegen abgelehnt (E205).
- Die Sperrzonen sichern Seitenkanten und Drehungen ab, die der Ultraschall (nur vorn/hinten) nicht sieht. **Beim Einrichten müssen alle Kanten manuell eingetragen werden.** (Auffälligkeit 21)

### 8.4 Stationen einlernen
1. UI → „Stations“ → „Teach station“. Roboter per Joystick in die Zielposition fahren: bei STORAGE zentriert unter einen Wagen, bei DOCK auf die Ladeplatte, Ausrichtung in Einfahrrichtung.
2. „Save pose“ ruft `/map/upsert_station` mit der aktuellen AMCL-Pose und dem Namen auf.
3. Die Anfahrpose ergibt sich als Punkt `approach_distance` (0,40 m) **hinter** der Pose, in der die Einfahrrichtung rückwärts verlängert wird. Das System prüft, ob die Anfahrpose frei ist (Costmap), sonst Warnung.
4. Belegung: beim Einlernen „Cart present: yes/no“. Danach ändert nur die Mission oder `/mission/set_cart_location` die Belegung.

---

## 9. Missionslogik

### 9.1 Betriebsarten
`NORMAL` (autonome Aufträge), `MAPPING` (Joystick + SLAM), `MANUAL` (Joystick im Normalbetrieb, Aufträge pausiert). Wechsel nur im Zustand IDLE oder LOCKED und ohne Wagen an Bord. MANUAL ist auch mit Wagen an Bord erlaubt.

### 9.2 Missionszustände (`MissionStatus.state`)
`BOOTING`, `LOCKED`, `LOCALIZING`, `IDLE`, `AWAIT_RESUME`, `GOTO_PICKUP`, `ENGAGE_CART`, `GOTO_DEST`, `RELEASE_CART`, `GOTO_POINT`, `GOTO_DOCK`, `DOCKING`, `CHARGING`, `WAITING_MAINS`, `UNDOCKING`, `HELD_WITH_CART`, `NEEDS_OPERATOR`, `PAUSED_SAFETY`, `MAPPING`, `MANUAL`.

| Von | Ereignis | Nach |
|---|---|---|
| BOOTING | Nano und Lidar bereit | LOCKED |
| LOCKED | Quittierung (Abschnitt 6.3) | LOCALIZING (bei `resume_mode = CANCEL_ORDER` wird der Auftrag verworfen) |
| LOCALIZING | Lokalisierung ok | AWAIT_RESUME, wenn ein Auftrag mit Status ACTIVE gespeichert ist und `resume_mode = RESUME`; sonst IDLE |
| AWAIT_RESUME | Schritt `PICKUP_NAV`/`DEST_NAV` | setzt am gespeicherten Schritt fort |
| AWAIT_RESUME | Schritt `ENGAGE`/`RELEASE` | NEEDS_OPERATOR (Meldung: „Robot restarted during cart handling. Check position, then choose a step.“) |
| IDLE | Auftrag in der Warteschlange und Akku ≥ `min_order_v` | GOTO_PICKUP bzw. GOTO_POINT |
| IDLE | Warteschlange leer, nicht an der Station | GOTO_DOCK (System-Auftrag Priorität 0) |
| GOTO_PICKUP | Anfahrpose erreicht (Toleranz 0,04 m / 0,08 rad) | ENGAGE_CART |
| ENGAGE_CART | Action erfolgreich | GOTO_DEST |
| GOTO_DEST | Anfahrpose des Ziels erreicht | RELEASE_CART |
| RELEASE_CART | Action erfolgreich | IDLE (Auftrag COMPLETED) |
| GOTO_DOCK | Anfahrpose der Station erreicht | DOCKING |
| DOCKING | Strom fließt | CHARGING |
| DOCKING | Kein Netz (Ladespannung fehlt) | WAITING_MAINS |
| CHARGING | Ladeende | IDLE (am Dock) |
| IDLE (am Dock) | neuer Auftrag | UNDOCKING → GOTO_PICKUP |
| beliebig | Sicherheitszustand ≠ ARMED | PAUSED_SAFETY (Schritt bleibt gespeichert), danach LOCKED-Dialog |
| beliebig im Auftrag | Fehler ohne Wagen | Auftrag DEFERRED, Alert, IDLE |
| beliebig im Auftrag | Fehler mit Wagen an Bord | HELD_WITH_CART |

### 9.3 Aufträge
- **Typen:** `TRANSPORT` (Quellstation → Zielstation), `GOTO_POINT` (freier Punkt, nur ohne Wagen), `GOTO_DOCK` (Laden).
- **Zustände:** `QUEUED`, `ACTIVE`, `DEFERRED`, `COMPLETED`, `CANCELLED`. Schritte (`step`): `PICKUP_NAV`, `ENGAGE`, `DEST_NAV`, `RELEASE`, `DONE`; `GOTO_*` nur `NAV`.
- **Sortierung:** Priorität absteigend, dann `created_at` aufsteigend. Kein Verdrängen.
- **Prüfung beim Anlegen** (`CreateOrder`):
  1. Quelle ist Station Typ STORAGE, enabled, mit Wagen belegt und nicht von einem anderen offenen Auftrag reserviert, sonst E301.
  2. Ziel ist Station Typ STORAGE, enabled, frei und nicht reserviert, sonst E302. Quelle = Ziel → E303. Unbekannte Station → E304.
  3. `GOTO_POINT`: Punkt in freier Zelle der Basiskarte, nicht in einer Sperrzone (E205), kein Wagen an Bord.
  4. Karte vorhanden (E310).
  Beim Start des Auftrags wird alles erneut gegen den tatsächlichen Zustand geprüft.
- **Reservierung:** Quelle und Ziel gelten ab ACTIVE oder QUEUED als reserviert.
- **Abbrechen:** QUEUED → sofort CANCELLED. ACTIVE ohne Wagen → Navigation abbrechen, CANCELLED. ACTIVE mit Wagen → Wagen wird zur **Quellstation zurückgebracht** (sie ist frei), dann CANCELLED; scheitert das, HELD_WITH_CART.
- **Wiederholen** (`RETRY`) für DEFERRED setzt `attempts` zurück und stellt QUEUED.
- **Akku:** Beginn eines Auftrags nur bei `v_batt ≥ min_order_v` (Platzhalter 11,8 V). Sonst bleibt der Auftrag QUEUED, der Roboter fährt zum Dock, Info E307.

### 9.4 Fehler mit Wagen (HELD_WITH_CART)
Roboter hält an, Alert mit Optionen: `RETRY` (Schritt wiederholen), `RETURN_TO_SOURCE`, `CHOOSE_OTHER_STATION` (UI-Auswahl freier Stationen), `MANUAL_TAKEOVER` (Joystick, danach Quittieren). Ohne Entscheidung bleibt der Roboter stehen. Während HELD_WITH_CART werden neue Aufträge angenommen, aber nicht gestartet.

### 9.5 Betreiber-Alerts und Fehlercodes
| Code | Name | Schwere | Auslöser | Bedienerhandlung |
|---|---|---|---|---|
| E101 | ESTOP_ACTIVE | ERROR | Not-Aus gedrückt | Taster lösen, quittieren |
| E102 | NOT_ARMED | WARN | Befehl bei gesperrtem Roboter, Arming ohne Rückmeldung | quittieren |
| E103 | NANO_LINK_LOST | ERROR | Bridge verliert Nano | USB prüfen |
| E104 | NANO_PROTO_MISMATCH | ERROR | Firmware-/Protokollversion falsch | Firmware flashen |
| E105 | CLIFF_DETECTED | WARN | Abgrund vorn/hinten | automatisch, nach 3× Auftrag DEFERRED |
| E106 | BUMPER_HIT | WARN | Bumper | automatisch, nach 3× Auftrag DEFERRED |
| E107 | HUB_TIMEOUT | ERROR | Endschalter nicht erreicht | Hub prüfen |
| E108 | HUB_LIMIT_CONFLICT | ERROR | beide Endschalter aktiv | Verdrahtung prüfen |
| E109 | LIDAR_LOST | ERROR | kein /scan seit 2 s | USB/Lidar prüfen |
| E110 | BATTERY_CRITICAL | ERROR | `v_batt < critical_v` | Laden |
| E111 | CHARGER_FAULT | ERROR | Ladespannung da, aber Strom < `charge_detect_ma` nach 5 s | Kontakte prüfen |
| E112 | ENCODER_FAULT | ERROR | Fahrbefehl, aber keine Ticks für 1 s | Motor/Encoder prüfen |
| E201 | LOCALIZATION_FAILED | ERROR | 7.3 Schritt 4 nicht abgeschlossen | Pose antippen |
| E202 | LOCALIZATION_LOST | ERROR | σ zu groß | automatisch neu |
| E203 | NO_PATH | WARN | Planer findet nichts | Hindernis entfernen |
| E204 | PATH_BLOCKED | WARN | nach 3 Wartezyklen blockiert | Hindernis entfernen, Auftrag wiederholen |
| E205 | GOAL_IN_KEEPOUT | WARN | Ziel in Sperrzone | Ziel ändern |
| E206 | NAV_TIMEOUT | WARN | `nav_timeout_s` 180 überschritten | wiederholen |
| E301 | SOURCE_EMPTY | WARN | Quelle ohne Wagen/reserviert | Belegung korrigieren |
| E302 | DEST_OCCUPIED | WARN | Ziel belegt/reserviert | anderes Ziel oder Belegung korrigieren |
| E303 | SAME_SOURCE_DEST | WARN | Quelle = Ziel | ändern |
| E304 | STATION_UNKNOWN | WARN | Station fehlt | Station anlegen |
| E305 | CART_ENGAGE_FAILED | ERROR | Einfahren/Heben gescheitert | HELD/NEEDS_OPERATOR |
| E306 | CART_RELEASE_FAILED | ERROR | Absetzen gescheitert | HELD/NEEDS_OPERATOR |
| E307 | BATTERY_TOO_LOW_FOR_ORDER | INFO | Akku unter `min_order_v` | wartet auf Ladung |
| E308 | ORDER_NOT_FOUND | WARN | ungültige ID | – |
| E309 | INVALID_STATE | WARN | Aktion im falschen Zustand | – |
| E310 | MAP_MISSING | ERROR | keine aktive Karte | kartieren |
| E311 | DOCK_FAILED | ERROR | 3 Dock-Versuche erfolglos | Station prüfen |
| E312 | DOCK_NOT_DEFINED | WARN | keine Station Typ DOCK | Station einlernen |
| E401 | DB_ERROR | ERROR | SQLite-Fehler | Neustart, Backup |
| E402 | CONFIG_ERROR | ERROR | YAML fehlerhaft | Konfiguration prüfen |
| E403 | MAP_SAVE_FAILED | ERROR | Speichern fehlgeschlagen | Speicherplatz |
| E404 | MODE_SWITCH_FAILED | ERROR | Dienst startet nicht neu | Logs prüfen |
Jeder Code hat in der UI einen Textschlüssel `error.E###.title`/`.hint` in allen drei Sprachen.

---

## 10. Docking-Aktionen (`robotros_docking`)

Alle Fahrten laufen mit `/cmd_vel_docking` (Priorität vor Nav2, hinter dem Collision Monitor, 7.3 Zeile 3). Die Strecke wird über die Odometrie gemessen (Differenz der Position seit Start, nicht über die Zeit).

### 10.1 `EngageCart(station_id)`
1. **Vorprüfung:** Roboter liegt im Fangbereich um die Anfahrpose (≤ 0,05 m, ≤ 0,10 rad). Sonst bis zu 2 Korrekturversuche über Nav2, danach E305.
2. Optional (M3, `cart_check_enabled`): Wagenbeine im Scan prüfen (Treffer in ±0,04 m um die erwarteten Beinpositionen ≥ 60 %), sonst E301-Warnung „cart not found“.
3. **Einfahren:** gerade vorwärts mit 0,05 m/s über `approach_distance` (Startwert 0,40 m). Buzzer aus, Blinker ein.
   - Bumper-Kontakt **vor** `approach_distance − 0,08 m` → Stopp, E305 (Hindernis).
   - Bumper-Kontakt im Fenster `approach_distance ± 0,03 m` gilt als Anschlag → Stopp.
   - Strecke erreicht, kein Kontakt → Stopp.
4. **Heben:** `$HUB UP` bis `limit_top` oder `hub_timeout_s`. Erfolg: Zustand `lifted`, Footprint und Profil CART, Wagenmaske an, Wagen in der Datenbank `CARRIED`.
5. Zeitüberschreitung: einmal senken (`DOWN` bis `limit_bottom`) und einen zweiten Versuch. Danach E107 → E305.
6. Ausfahren: rückwärts `approach_distance` mit 0,05 m/s, Buzzer REVERSE, hinterer Ultraschall überwacht. Der Roboter steht danach in der Anfahrpose und übergibt an Nav2.

### 10.2 `ReleaseCart(station_id)`
1. Vorprüfung wie 10.1 mit Wagenfootprint.
2. Einfahren wie 10.1 (Wagen mit angehobenen Stiften in die Führung), Anschlag/Strecke wie dort.
3. **Senken:** `$HUB DOWN` bis `limit_bottom`. Erfolg → Wagen in der Datenbank `STORED` an dieser Station, `lifted = false`, Footprint auf Roboter, Wagenmaske aus.
4. Ausfahren rückwärts auf die Anfahrpose.
5. Fehler: nicht unten → E107 → E306 (Zustand HELD_WITH_CART, weil der Wagen noch am Roboter ist).

### 10.3 `DockCharger(station_id)`
1. Vorprüfung (Anfahrpose ≤ 0,05 m / 0,10 rad).
2. Geradeaus mit 0,05 m/s über `approach_distance` (Startwert 0,40 m); Plattenkontakt wird durch **Ladespannung am Roboter** (`v_charger ≥ charger_present_v` 12,0 V) erkannt. Wird sie erreicht, nach weiteren 0,02 m stoppen. Wird sie nicht erreicht, endet die Fahrt an der Solllänge.
3. Keine Ladespannung nach 3 s: Fall „kein Netz“ möglich → Zustand WAITING_MAINS (bleibt an der Position) und alle 10 s prüfen. Gibt es trotz Netz keine Spannung, 3 Versuche (0,25 m zurück, neu anfahren), dann E311.
4. Ladespannung da: Laderelais an, innerhalb von 5 s muss `i_charge ≥ charge_detect_ma` (50 mA) fließen, sonst Relais aus, E111.

### 10.4 `Undock`
Laderelais aus, 0,25 m rückwärts (`undock_travel_m`) mit 0,05 m/s, Buzzer REVERSE, hinterer Ultraschall aktiv; danach Anfahrpose als freie Fläche prüfen.

### 10.5 `Localize` — siehe 7.3

---

## 11. Laden und Energie (`charging_manager`)

- **Messgrößen:** `v_batt` (gefiltert, Median über 60 s, nur ruhende Phasen zählen für Schwellen), `v_charger`, `i_charge` (INA219, im Ladepfad zwischen Kontakt/Relais und Akku), `v_rail` (Motor-/Sicherheitsschiene, Diagnose).
- **Platzhalter bis Akkutest** (C9), alle in `power.yaml`: `low_battery_v` 11,8 · `critical_v` 11,2 · `shutdown_v` 10,8 · `min_order_v` 11,8 · `charge_detect_ma` 50 · `charge_end_ma` 50 (Ladeende, wenn `i_charge` ≤ Wert über 60 s bei `v_batt ≥ 13,5 V`) · `charge_max_h` 8 · `recharge_v` 12,6 (Laderelais erneut schließen) · `charger_present_v` 12,0.
- **Auslöser Laden:** (a) Auftrag fertig und Warteschlange leer, (b) `v_batt < low_battery_v`: aktuellen Auftrag beenden, dann GOTO_DOCK.
- **Kritisch:** `v_batt < critical_v` → keine neuen Aufträge, nach dem aktuellen Auftrag Dock (E110). Unter `shutdown_v` Stopp, Alert, nach 60 s `systemctl poweroff`.
- **Ladegerät-Anforderung:** REQ-ELEC-02 (13,8 V CV, ≤ 0,72 A, Strombegrenzung, Verpolschutz). Das Ladeende erkennt der Roboter selbst, das Laderelais trennt dann.

---

## 12. Datenmodell (SQLite, `/var/lib/robotros/robotros.db`)

Einstellungen: `PRAGMA journal_mode=WAL`, `PRAGMA foreign_keys=ON`, `PRAGMA synchronous=FULL` (Stromausfall). Schema-Version in `settings(key='schema_version')`, Migrationen als nummerierte SQL-Dateien im Paket `robotros_mission/sql/`. Täglicher Backup nach `/var/lib/robotros/backup/`.

```sql
CREATE TABLE maps (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL, yaml_path TEXT NOT NULL,
  resolution REAL, created_at TEXT, is_active INTEGER DEFAULT 0);
CREATE TABLE stations (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id), name TEXT NOT NULL,
  kind TEXT NOT NULL CHECK(kind IN ('STORAGE','DOCK')), x REAL, y REAL, yaw REAL,
  approach_distance REAL DEFAULT 0.40, enabled INTEGER DEFAULT 1, UNIQUE(map_id, name));
CREATE TABLE carts (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL,
  station_id INTEGER REFERENCES stations(id),            -- NULL während des Transports
  state TEXT NOT NULL CHECK(state IN ('STORED','CARRIED','UNKNOWN')));
CREATE TABLE orders (id INTEGER PRIMARY KEY AUTOINCREMENT,
  type TEXT NOT NULL CHECK(type IN ('TRANSPORT','GOTO_POINT','GOTO_DOCK')),
  origin TEXT DEFAULT 'USER', priority INTEGER DEFAULT 5 CHECK(priority BETWEEN 0 AND 9),
  source_station_id INTEGER REFERENCES stations(id), dest_station_id INTEGER REFERENCES stations(id),
  goal_x REAL, goal_y REAL, goal_yaw REAL, cart_id INTEGER REFERENCES carts(id),
  state TEXT NOT NULL, step TEXT, error_code TEXT, error_detail TEXT, attempts INTEGER DEFAULT 0,
  created_at TEXT, started_at TEXT, finished_at TEXT);
CREATE TABLE order_events (id INTEGER PRIMARY KEY AUTOINCREMENT, order_id INTEGER REFERENCES orders(id),
  ts TEXT, event TEXT, detail TEXT);
CREATE TABLE keepout_zones (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id),
  source TEXT NOT NULL CHECK(source IN ('MANUAL','CLIFF_AUTO')), polygon_json TEXT NOT NULL, note TEXT, created_at TEXT);
CREATE TABLE contact_points (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id),
  x REAL, y REAL, hits INTEGER DEFAULT 1, permanent INTEGER DEFAULT 0, first_ts TEXT, last_ts TEXT);
CREATE TABLE robot_state (id INTEGER PRIMARY KEY CHECK(id=1), lifted INTEGER DEFAULT 0, active_order_id INTEGER,
  last_x REAL, last_y REAL, last_yaw REAL, updated_at TEXT);
CREATE TABLE settings (key TEXT PRIMARY KEY, value TEXT);
```
Belegung einer Station = es gibt einen Wagen mit `station_id = stations.id`. Temporäre Kontaktpunkte (nicht dauerhaft) liegen nur im Speicher. `robot_state` wird bei jedem Schrittwechsel geschrieben. Nach dem Boot: Ist `limit_top` aktiv, fragt der Dialog „Is a cart on board?“ (Ja → `lifted = 1`, Wagen `CARRIED`; Nein → Hub senken, Wagenstatus UNKNOWN).

---

## 13. Web-UI

### 13.1 Rahmen
- Querformat, nur Touch, Mindestgröße Touchziele 56 px, Schrift ≥ 18 px, Auslegung für 1368 × 912 CSS-Pixel (Skalierung des Surface möglich).
- Statisch ausgeliefert vom Pi (`http://robotros.local:8080`, Hostname `robotros` per avahi), rosbridge-URL = `ws://` + `window.location.hostname` + `:9090` (keine feste IP im Code).
- Kein Build-Schritt: ES-Module, `roslib.min.js` und `eventemitter2` unter `web_ui/vendor/` (offline).
- Struktur: `web_ui/index.html`, `css/`, `js/{main,ros,state,map_view,i18n}.js`, `js/pages/*.js`, `i18n/{en,de,it}.json`, `vendor/`.

### 13.2 Kopfzeile (immer sichtbar)
Verbindungsstatus (rosbridge), Sicherheitszustand, Betriebsart und Missionszustand, Akku (Spannung, Symbol, Ladeanzeige), Lokalisierungsgüte, aktiver Auftrag (Schritt, Fortschritt), **großer roter Not-Stopp-Button** rechts oben (ruft `/mission/software_estop`). Verbindungsverlust: Overlay „Connection lost – robot continues its work autonomously“, Wiederverbindung mit 1 → 5 s Backoff, danach `/map/get_snapshot`.

### 13.3 Seiten
| Seite | Inhalt |
|---|---|
| **Map** (Start) | Karte, Roboterpose (Pfeil), Plan, Stationen (Symbole mit Belegung), Sperrzonen, Kontaktpunkte; Antippen eines Ziels (freier Punkt) → Bestätigen → `GOTO_POINT`. |
| **Orders** | Neuer Auftrag: Quelle (nur belegte Stationen), Ziel (nur freie), Priorität (Schieber 0–9, Standard 5), „Start“. Liste mit Zustand, Schritt, Fehler; Aktionen Abbrechen, Wiederholen, Priorität ändern, Fortsetzen ab Schritt. |
| **Stations** | Liste, Einlernen (8.4), Belegung korrigieren, Station aktivieren/deaktivieren. |
| **Zones** | Sperrzonen zeichnen und löschen, Kontaktpunkte löschen, Touch-Mapping ein/aus. |
| **Manual** | Zwei virtuelle Sticks oder ein Stick (Fahren/Drehen), Totmann (loslassen = Stopp, 20 Hz, Timeout 300 ms), Schaltflächen Hub hoch/runter (nur MANUAL), Langsam-Modus, Scan-Overlay. |
| **Mapping** | Start/Stop Kartieren, Live-Karte, „Save map“ mit Namen. |
| **System** | Akku/Spannungen, Sensorzustände (Bumper, Ultraschall, Endschalter), Firmwareversion, Sprache, Fehlerverlauf, Neustart des Dienstes, Herunterfahren des Pi. |
| **Dialoge** | Quittierung (6.3), Alerts (`OperatorAlert`), Neustartbestätigung, Pose antippen (7.3). |

### 13.4 i18n
`data-i18n="key"`, flache Schlüssel (`order.create.title`). Fehlende Schlüssel zeigen den Schlüssel selbst (kein Absturz). Sprache in `localStorage` (try/catch), Standard `en`. Test `tools/check_i18n.py`: jeder Schlüssel der EN-Datei muss in DE und IT existieren, alle `error.E###.*` vorhanden.

---

## 14. Simulation (`robotros_sim`)

- **Welt:** 7 × 7 m, Wände 0,10 m. Drei Abstellplätze S1–S3 an Wänden mit Führungsschienen, Ladeplatz D in einer Ecke, zwei Wagen C1, C2 (4 Stützen 0,02 m, Deckplatte 0,30 × 0,30 m auf Höhe `cart_clearance`), ein **Abgrundbereich** E (Polygon aus der Weltdatei: der Sim-Adapter meldet „kein Echo“ und bremst, die Physik bleibt flach), eine **Glaswand** (nur Kollisionsgeometrie, **ohne Visual**, daher für den GPU-Lidar unsichtbar, aber für den Kontaktsensor spürbar), ein enger Durchgang 0,45 m, ein statisches Hindernis, eine **bewegte Person** (Zylinder 0,4 m, Waypoints per Skript `sim_person`, Pausen konfigurierbar).
- **Roboter:** Gazebo-Diff-Drive (`gz-sim-diff-drive-system`), GPU-Lidar 720 Strahlen, 0,15–12 m, 10 Hz, Rauschen σ 0,01 m (A1-nah), Kontaktsensor vorn in 3 Zonen, Ultraschall als schmale Strahlen mit Klassifizierung nach Weltgeometrie, Hub als Prismatik-Gelenk mit Endschaltern, **Wagen wird über ein `DetachableJoint` gekoppelt**, wenn Hub oben und die Posenabweichung innerhalb `engage_tolerance` (0,02 m) liegt (kinematische Näherung).
- **`sim_hw_adapter`:** veröffentlicht dieselben `/hw/*`-Topics wie die Bridge (Sicherheitszustand identisch zur Nano-Logik, Akkumodell mit Spannungskurve und Ladung, Hubzustand). Die Bridge selbst läuft nicht in der Simulation, aber `fake_nano` testet sie separat.
- **`fake_nano`:** Python-Prozess hinter einem Pseudo-Terminal (pty), spricht das echte Protokoll (`spez_firmware_protokoll.md`), simuliert Motoren (PT1), Encoder, Ultraschall, Bumper, Not-Aus, Hub. Skriptbare Ereignisse für Tests.
- Alle Knoten laufen mit `use_sim_time:=true`.
- Startkommando: `ros2 launch robotros_sim sim.launch.py world:=generic mode:=nav`.
- Später (falls Hardware sehr spät): Welt `real_room.sdf` nach Messung des Raums.

---

## 15. Betrieb auf Pi und Surface

### 15.1 Pi 5
- **Dienst** `robotros.service` (User `robotros`, `Restart=on-failure`, `RestartSec=5`, `After=network.target`, **nicht** `network-online`, damit der Roboter ohne WLAN startet). `ExecStart=/opt/robotros/bin/run_robot.sh`, liest `/var/lib/robotros/mode.env`.
- **Umgebung:** `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Entwicklung: `SUBNET` über `ROBOTROS_DEV=1`). Cyclone-Konfiguration `deploy/cyclonedds.xml` mit erhöhtem `MaxAutoParticipantIndex` (100) **[geprüft: Standard in Jazzy ca. 32, bei vielen Prozessen kann „no free participant index“ auftreten]**.
- **udev:** `/dev/robotros_nano_drive`, `/dev/robotros_nano_aux`, `/dev/rplidar` über Seriennummern (`deploy/99-robotros.rules`). USB-IDs für Nano Every und den CP2102 des Lidar per `lsusb` bestätigen **[prüfen]**.
- **Sudoers-Eintrag** (nur für `systemctl restart/stop robotros.service` und `poweroff`) für die Funktionen Moduswechsel und Herunterfahren.
- **Zeit:** chrony im Slew-Modus (`makestep 0 0`), Logik nur mit Monotonic-Zeit, Zeitstempel in der DB sind informativ. [D-21]
- **Netzwerk:** Hostname `robotros`, avahi; DHCP-Reservierung im Router (falls möglich); kein Internet nötig.
- **Logs:** journald, Rotation 7 Tage, Fehlerzusammenfassung in `order_events`.
- **Speicher:** Pi-5-microSD wird bei Strom aus riskant: Dateisystem ext4 mit `commit=5`, SQLite `synchronous=FULL`; **Empfehlung:** hochwertige A2-Karte oder NVMe-SSD (Nachkaufliste).

### 15.2 Surface
- Zusätzlich zu Ubuntu 24.04: **linux-surface-Kernel und `iptsd` sind für den Touch des Surface Pro 7 nötig** (Ubuntu 24.04 liefert keine Touch-Treiber, die Vorgabe „nur Touch“ macht das zur Pflicht). **[geprüft: Foren und Projektseite bestätigen, dass das Surface Pro 7 den linux-surface-Kernel braucht; der Stift ist bei Pro 7 teilweise problematisch, wird aber nicht benötigt]**
- Browser: Firefox im Kiosk-Modus (`firefox --kiosk http://robotros.local:8080`), Autostart nach Anmeldung, Bildschirmschoner/Energiesparen für den Bildschirm aus, Bildschirmrotation fest Querformat.
- Optional (Entwicklung): ROS 2 Jazzy Desktop, Gazebo Harmonic, RViz2.
- Check-Skript `tools/check_surface.sh` prüft Touch, WLAN, Browser-Start, Verbindung zum Pi.

---

## 16. Repository und Arbeitsweise

```
RobotROS/
├─ README.md                 (Englisch: Überblick, Quickstart)
├─ ros2_ws/src/              (Pakete aus 3.2)
├─ firmware/                 (PlatformIO: platformio.ini, include/pins_*.h, lib/core, src/, test/, test_sketches/)
├─ web_ui/                   (13.1)
├─ deploy/                   (systemd, udev, sudoers, chrony, cyclonedds.xml, run_robot.sh)
├─ tools/                    (Kalibrier- und Hilfsskripte, check_i18n.py, check_surface.sh)
├─ docs/{it,de,en}/          (Relazione tecnica, Anleitungen, Testprotokolle) und docs/adr/ (Entscheidungen)
└─ .github/workflows/ci.yml
```
- **Branching:** `main` immer lauffähig, Arbeit auf `feature/<thema>`, Pull Request mit Selbstprüfung, mindestens ein Teammitglied sieht jede Änderung an Sicherheitslogik.
- **Commits/PRs** Englisch, kurze Betreffzeile. Commits, die Claude erstellt, enden mit den Attributionszeilen aus den Sitzungsvorgaben.
- **Code-Stil:** Python mit `ruff` und Typannotationen, C++ mit `clang-format`, ROS-Parameter immer deklariert mit Standardwerten, Logik von Knotenklassen getrennt (`core/*.py`), damit Unit-Tests ohne ROS laufen.
- **CI (GitHub Actions):** `colcon build` und `colcon test` im Docker-Image `ros:jazzy`, `ruff`, PlatformIO `native`-Tests (Firmware-Kernlogik auf dem Host), `check_i18n.py`.
- **Dokumentation** entsteht parallel zum Code (`docs/adr/` pro Entscheidung, Testprotokolle aus `spez_parameter_tests.md`).
- Das Repository existiert noch nicht → Anleitung `claude/anleitung_github.md`. Bis dahin liegen alle Spezifikationen im Projekt.

---

## 17. Reihenfolge der Entwicklung (ohne Hardware beginnend)

| Phase | Inhalt | Hardware nötig |
|---|---|---|
| 0 | Pi- und Surface-Setup (Ubuntu, ROS 2 Jazzy, linux-surface, Firefox-Kiosk) | Pi, Surface |
| 1 | Repository, Interfaces, Protokollbibliothek, Firmware-Kern mit Host-Tests, `fake_nano` | keine |
| 2 | Simulation (Welt, Roboter, Adapter), Bridge gegen `fake_nano`, Nav2/SLAM/AMCL in der Sim, Web-UI mit Karte/Joystick | keine |
| 3 | **M1 in Sim:** kartieren, A→B, Hindernisse, Person, Keepout, Lokalisierung | keine |
| 4 | Firmware auf Nano Every (Stand-in: LEDs, Taster, Poti), HIL-Test Protokoll | Nano Every |
| 5 | **M1 real** (nach Lieferung: Motoren, Lidar, Sensoren, Akku): Kalibrierung C1–C8, Sicherheitstests | Lieferung |
| 6 | **M2:** Hub, Wagen, Stationen, Engage/Release in Sim, dann real | Hub, Wagen |
| 7 | **M3:** Warteschlange, Laden/Docking, Netz-/Neustart-Tests, Touch-Mapping, Fehlertabelle, Politur | Ladestation |
| 8 | Dokumentation IT/DE, dann EN, Präsentation, Demo-Probe | – |
Meilenstein-Abnahmekriterien: `spez_parameter_tests.md` §6.

---

## 18. Anforderungen an Mechanik und Elektrik (REQ)

| ID | Anforderung | Grund |
|---|---|---|
| REQ-MECH-01 | Wagenaufnahme: Lochdurchmesser/Führung muss ±15 mm Längsabweichung und ±10 mm Querabweichung sowie ±5° Winkel des Roboters beim Einfahren ausgleichen (Einführschrägen). | Odometrie und Anfahrpose |
| REQ-MECH-02 | Die Lidar-Scanebene muss **unter der Wagenunterseite** liegen. Im Scanbereich des Roboters unter dem Wagen dürfen nur die vier Stützen liegen, keine Seitenwände oder Querstreben. | Wagenmaske, Lokalisierung bei Transport |
| REQ-MECH-03 | Hubwinde muss **selbsthemmend** sein (Schnecke oder Haltebremse), Hubhöhe 2–3 cm, Endschalter oben und unten. | Not-Aus bei angehobenem Wagen |
| REQ-MECH-04 | Ladeplatte: aktive Kontaktfläche ≥ Kontaktabstand + 80 mm breit, ≥ 100 mm lang in Fahrtrichtung; flach (≤ 3 mm), Einführschrägen. | Anfahrtoleranz ±40 mm / 5° |
| REQ-MECH-05 | **Breitenprüfung:** Roboterbreite + 2 × 30 mm ≤ lichte Weite zwischen den Wagenbeinen. Bei Wagen 300 mm außen und Beinen 20 mm ergibt das **≤ ca. 200 mm Roboterbreite**. Die Platzhaltermaße 0,26 m müssen im CAD angepasst oder die Wagenbeine nach innen versetzt/schmaler werden. | Roboter muss zwischen den Beinen einfahren |
| REQ-MECH-06 | Vorderer Ultraschall: Bodenpunkt ≥ 80 mm vor dem äußersten Bodenkontakt (Stützrad/Rad) in Fahrtrichtung, hinterer ebenso. Bodenneigung (Montagewinkel) im CAD festlegen und als `us_pitch_deg` eintragen. | Bremsweg bei 0,15 m/s: ca. 50 mm (siehe Firmware-Spec §6) |
| REQ-MECH-07 | Stationen und Ladeplatz dürfen keine Vertiefungen größer als die Abgrundschwelle (`cliff_margin_mm` 40) im Messbereich des Ultraschalls haben. | Fehlalarme vermeiden |
| REQ-MECH-08 | Bumper: drei Endschalter (links/mitte/rechts), Schaltweg ≤ 5 mm, Hebel so, dass jede Zone ≥ 0,08 m Breite überdeckt; Anschlag verhindert Überlast. | Kontaktpunkte |
| REQ-ELEC-01 | **Alle Relaiseingänge haben externe Pull-Widerstände auf den Ruhepegel**, sodass das Sicherheitsrelais bei Nano-Reset oder fehlender Firmware **abfällt**. Not-Aus-Öffner liegt in der Spulenversorgung (`pinplan_nano.md` §8). | Ausfallsicherheit |
| REQ-ELEC-02 | Ladegerät: 13,8 V CV (±0,2 V), Strombegrenzung ≤ 0,72 A, Verpolschutz (Diode), keine offenen Spannungsflächen größer als nötig. | Datenblatt Akku |
| REQ-ELEC-03 | 5-V-Versorgung in **zwei getrennten Schienen**: „no safety“ (Logik, Sensoren, Nano, Lidar über Pi/USB) ≥ 2 A und „safety“ (Motoren, Hub) ≥ 3 A, je ein Wandler. Lidar und Pi **nicht** auf der Motorschiene. | Störungen, Lidar-Welligkeit ≤ 50 mV |
| REQ-ELEC-04 | INA219 im Ladepfad zwischen Kontakt/Relais und Akku (Shunt 0,1 Ω), Messbereich ±3,2 A. | Ladeende |
| REQ-ELEC-05 | Sicherungen: Akku-Hauptsicherung 5 A, je Schiene eine Sicherung. | Blei-Akku kann hohe Ströme liefern |
| REQ-ELEC-06 | Pi 5: stabile 5 V/5 A empfohlen. Falls die PiEnergy-Mini nur 3 A liefert, Meldung „USB power limited“ bei Lidar-Start beobachten und Lidar notfalls über eigene Schiene versorgen. | Pi-5-USB-Strombegrenzung |

---

## 19. Änderungsprotokoll dieser Spezifikation
| Datum | Änderung | Von |
|---|---|---|
| 18.11.2026 | Erstfassung nach Runde 25 und Delegation | Claude |
