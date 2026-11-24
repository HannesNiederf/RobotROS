# Spezifikation Nano-Firmware und serielles Protokoll „robotROS“ v1.0

Stand: 18.11.2026 · Teil der Softwarespezifikation (`claude/softwarespezifikation.md`, dort D-01 bis D-28)
Zielhardware: **Arduino Nano Every (ATmega4809, 20 MHz, 48 KB Flash, 6 KB SRAM, 5 V)**, Toolchain PlatformIO (`platform = atmelmegaavr`, `board = nano_every`, `framework = arduino`) [Wissen, beim Anlegen des Projekts bestätigen]
Legende wie dort. Pin-Nummern stehen **nicht** hier, sondern im Pin-Header je Variante (Quelle: `claude/pinplan_nano.md`, Variante A, Tabellen Board A und B).

---

## 1. Rollen und Pin-Abstraktion

### 1.1 Rollen
| Rolle | Code | Aufgaben | Entspricht (Variante A) |
|---|---|---|---|
| `DRIVE` | 1 | 2 Motoren (L298N #1), 2 Encoder, Ultraschall vorn/hinten, 3 Bumper, Not-Aus-Abfrage, Arming-Ausgang (Sicherheitsrelais), Sicherheitszustandsautomat, Watchdog | Board A |
| `AUX` | 2 | Hubmotor (L298N #2), 2 Endschalter, 3 NeoPixel-Sticks, Buzzer, Relais (Laden, Power, Reserve), 3 Spannungssensoren (Analog), INA219 (I²C) | Board B |
| `ALL` | 3 | Vereinigung (Variante mit Expander oder Schieberegistern) | – |

Eine Firmware, drei Build-Ziele (`pio run -e drive`, `-e aux`, `-e all`). Die Rolle wird per Compiler-Flag (`-DROLE_DRIVE` usw.) gewählt, nicht zur Laufzeit.

### 1.2 Pin-Header
`firmware/include/pins_<variante>_<rolle>.h` enthält ausschließlich Namen und Pegel. Pflichtnamen:
- DRIVE: `PIN_MOTL_PWM, PIN_MOTL_IN1, PIN_MOTL_IN2, PIN_MOTR_PWM, PIN_MOTR_IN1, PIN_MOTR_IN2, PIN_ENCL_A, PIN_ENCL_B, PIN_ENCR_A, PIN_ENCR_B, PIN_US_FRONT, PIN_US_REAR, PIN_BUMP_L, PIN_BUMP_M, PIN_BUMP_R, PIN_ESTOP_SENSE, PIN_ARM_OUT`
- AUX: `PIN_HUB_PWM, PIN_HUB_IN1, PIN_HUB_IN2, PIN_HUB_LIM_TOP, PIN_HUB_LIM_BOT, PIN_LED_L, PIN_LED_R, PIN_LED_REAR, PIN_BUZZER, PIN_REL_CHARGE, PIN_REL_POWER, PIN_REL_RESERVE, PIN_VS_BATT, PIN_VS_CHG, PIN_VS_RAIL` (I²C fest A4/A5)
- Pegel/Polaritäten: `ESTOP_PRESSED_LEVEL`, `BUMP_ACTIVE_LEVEL` (Standard LOW mit INPUT_PULLUP), `LIMIT_ACTIVE_LEVEL`, `RELAY_ACTIVE_LEVEL_CHARGE/_POWER/_RESERVE`, `ARM_ACTIVE_LEVEL`, `MOT_INVERT_L/R`, `ENC_INVERT_L/R`.
Ein Wechsel auf Expander/Schieberegister ersetzt nur die `digitalRead/Write`-Aufrufe in der Hardware-Schicht (`hal.h`), nicht die Logik.

### 1.3 Polaritäten, die erst am Gerät feststehen
Pololu-Relaismodule (Ansteuerpegel und Verhalten bei Reset), Not-Aus-Taster (Öffner/Schließer), Endschalter (NO/NC) sind **noch nicht aus Datenblättern belegt**. Deshalb: Pegelkonstanten im Header, Bring-up-Sketch `relay_probe` (Abschnitt 11.3) ermittelt sie vor dem ersten Einbau, Ergebnis in den Header und in `docs/`.

---

## 2. Serielles Protokoll (Version 1)

### 2.1 Rahmen
- USB-CDC des Nano Every an den Pi, **115200 Baud, 8N1**. Zeichenkodierung ASCII.
- Ein Telegramm: `$` Nutzdaten `*` Prüfsumme `\n`
  - Nutzdaten: `BEFEHL,feld1,feld2,…`, nur druckbare Zeichen, keine Leerzeichen.
  - **Prüfsumme:** XOR aller Bytes zwischen `$` und `*` (ohne diese), zwei Hexziffern in Großbuchstaben.
  - Abschluss `\n` (LF). `\r` wird ignoriert.
  - Maximal 96 Zeichen je Telegramm inklusive Rahmen. Längere Zeilen werden verworfen, `$ERR,6`.
  - Zahlen: ganze Dezimalzahlen mit optionalem Minus. **Keine Fließkommazahlen** (skalierte Ganzzahlen).
- Ungültige Prüfsumme: Telegramm verwerfen, `$ERR,1,<befehl>`. Der Pi sendet zyklische Befehle (VEL) ohnehin erneut.
- **Protokollversion:** `PROTO_VER = 1`. Der Pi prüft sie im `$HELLO`.
- Beispiel (Prüfsumme als Platzhalter, die echten Werte kommen aus den Testvektoren): `$VEL,12,300,-300*HH`

### 2.2 Pi → Nano
| Telegramm | Rolle | Bedeutung | Antwort |
|---|---|---|---|
| `$WHO` | alle | Identifikation anfordern | `$HELLO` |
| `$PING,<seq>` | alle | Lebenszeichen ohne Fahrbefehl | `$PONG,<seq>` |
| `$VEL,<seq>,<l_tps>,<r_tps>` | DRIVE | Solldrehzahl der Räder in Ticks/s, signiert, Begrenzung `TPS_MAX` | keine (zyklisch 20 Hz) |
| `$ARM,<0\|1>` | DRIVE | 1 = Arming anfordern, 0 = Disarm | `$ACK,ARM` oder `$ERR,5` |
| `$CLR` | alle | verriegelte Fehler löschen (Hubfehler) | `$ACK,CLR` |
| `$CFG,<schlüssel>,<wert>` | je nach Schlüssel | Parameter setzen (Abschnitt 10) | `$ACK,CFG,<schlüssel>` oder `$ERR,3` |
| `$HUB,<kommando>,<pwm>` | AUX | 0 Stopp, 1 hoch, 2 runter, `pwm` 0–255. **Lease:** läuft nur, solange mindestens alle 500 ms ein `$HUB` kommt | `$HUBST` |
| `$LED,<streifen>,<muster>,<r>,<g>,<b>` | AUX | Streifen 0 links, 1 rechts, 2 hinten; Muster 0 aus, 1 dauerhaft, 2 blinkend, 3 Lauflicht | `$ACK,LED` |
| `$BUZ,<muster>` | AUX | 0 aus, 1 Rückwärts (400 ms Takt), 2 kurz, 3 Alarm | `$ACK,BUZ` |
| `$REL,<kanal>,<0\|1>` | AUX | 1 Laden, 2 Power, 3 Reserve | `$ACK,REL,<kanal>` |

### 2.3 Nano → Pi
| Telegramm | Rolle | Rate | Felder |
|---|---|---|---|
| `$HELLO,<proto>,<fw>,<rolle>,<caps>` | alle | beim Start, danach alle 2 s, bis der erste gültige Pi-Befehl ankommt; außerdem auf `$WHO` | `fw` z. B. `1.0.0`; `caps` Bitmaske: bit0 Encoder, bit1 Ultraschall, bit2 Bumper, bit3 Hub, bit4 LED, bit5 Relais, bit6 Spannung, bit7 INA219 gefunden |
| `$PONG,<seq>` | alle | auf Anfrage | |
| `$ODO,<t_ms>,<l_ticks>,<r_ticks>,<l_tps>,<r_tps>` | DRIVE | 50 Hz | kumulative Ticks `int32`, gemessene Rate Ticks/s |
| `$SEN,<t_ms>,<usF_mm>,<usR_mm>,<clsF>,<clsR>,<bump>` | DRIVE | ca. 16 Hz | Abstand in mm (65535 = kein Echo), Klasse 0 unbekannt/1 Boden/2 Abgrund/3 Hindernis, `bump` Bits: 1 links, 2 mitte, 4 rechts |
| `$SAF,<t_ms>,<zustand>,<flags>` | DRIVE | 5 Hz und bei Änderung | Zustand 0 BOOT_LOCKED, 1 ESTOP, 2 WAIT_ACK, 3 ARMED. Flags: bit0 Not-Aus gedrückt, bit1 Relais an, bit2 Vorwärts gesperrt, bit3 Rückwärts gesperrt, bit4 Pi-Verbindung verloren, bit5 Software-Stopp, bit6 Abgrund vorn, bit7 Abgrund hinten, bit8 Hubfehler |
| `$PWR,<t_ms>,<adc_batt_mv>,<adc_chg_mv>,<adc_rail_mv>,<i_ma>` | AUX | 2 Hz | ADC-Spannung am Pin in mV (der Pi rechnet mit Teilerfaktor und Kalibrierung), Strom in mA signiert (positiv = lädt), −32768 wenn INA219 fehlt |
| `$HUBST,<t_ms>,<zustand>,<endschalter>,<fehler>` | AUX | bei Änderung und 2 Hz während der Fahrt | Zustand 0 Ruhe, 1 hoch, 2 runter, 3 oben, 4 unten, 5 Fehler; Endschalter Bit0 oben, Bit1 unten; Fehler 0 keiner, 1 Zeitüberschreitung, 2 beide Endschalter aktiv |
| `$EVT,<t_ms>,<typ>,<arg>` | alle | bei Ereignis | Typ 10 Not-Aus gedrückt, 11 Not-Aus gelöst, 20 Abgrund vorn an, 21 aus, 22 Abgrund hinten an, 23 aus, 30 Bumper-Maske geändert (arg = Maske), 40 Hub oben, 41 Hub unten, 42 Hub-Zeitüberschreitung, 50 Pi-Verbindung verloren, 51 wieder da |
| `$ACK,<befehl>[,<arg>]` | alle | auf Befehle | |
| `$ERR,<code>[,<detail>]` | alle | bei Fehler | 1 Prüfsumme, 2 unbekannter Befehl, 3 ungültiges Argument, 4 von der Rolle nicht unterstützt, 5 im Zustand nicht erlaubt, 6 Zeile zu lang, 7 beschäftigt |
`t_ms` = `millis()` des Nano (32 Bit, läuft nach ca. 49 Tagen über, die Bridge rechnet mit Differenzen).

### 2.4 Bandbreite (Rolle ALL, ungünstigster Fall)
ODO 50 Hz × ca. 40 Byte = 2,0 kB/s, SEN 16 Hz × 45 = 0,7 kB/s, SAF 5 Hz × 25 = 0,1 kB/s, PWR 2 Hz × 40, HUBST 2 Hz × 35 = 0,2 kB/s, Befehle ca. 1 kB/s. Summe ca. 4 kB/s bei 11,5 kB/s Kapazität (115200 Baud). Reserve > 60 %.

### 2.5 Versionierung und Kompatibilität
Das Feld `proto` im HELLO muss mit dem Wert in der Bridge übereinstimmen, sonst E104. Neue optionale Felder werden nur **am Ende** angehängt, der Parser ignoriert zusätzliche Felder. Eine inkompatible Änderung erhöht `PROTO_VER`.

### 2.6 Verhalten der Bridge (Pi)
1. Geräteliste aus `bridge.yaml` (`devices: [{name, port, role, baud}]`), z. B. `drive → /dev/robotros_nano_drive`, `aux → /dev/robotros_nano_aux`.
2. Verbindung öffnen, `$WHO` senden, auf `$HELLO` bis 1 s warten, `proto` und `role` prüfen, dann **alle CFG-Werte senden** (aus `firmware.yaml`), jeweils auf `$ACK` warten.
3. Routing nach Rolle (Befehlstabelle 2.2). Befehle für eine fehlende Rolle werden mit Warnung verworfen.
4. Zyklisch: `$VEL` mit 20 Hz, `$HUB` mit 10 Hz solange ein Hubbefehl aktiv ist, `$PING` mit 5 Hz an AUX.
5. Verbindungsverlust (kein Byte > 300 ms oder Lesefehler): Zustand `link_lost`, Wiederverbindung alle 1 s, `SafetyState.pi_link_lost` im Topic, Alert E103.
6. **Unerwartetes `$HELLO` im Betrieb = Nano wurde zurückgesetzt:** CFG neu senden, Zustand wie BOOT_LOCKED behandeln, Mission pausiert und verlangt Quittierung.
7. Odometrie: Differenz der Ticks (32-Bit-Arithmetik mit Überlauf), Zeitbasis aus `t_ms`-Differenz, Geometrie aus `robot.yaml`. Positionsfortschreibung: Differenzialantrieb mit Mittelpunktsintegration. `geometry_calibrated: false` → Geschwindigkeit hart auf 0,05 m/s begrenzen (D-24).
8. Ein Telegrammfehler (Prüfsumme) wird gezählt (`/diagnostics`), ab 5 % Fehlerrate Warnung.

---

## 3. Firmware-Architektur

### 3.1 Module
| Modul | Datei(en) | Inhalt |
|---|---|---|
| Core (ohne Arduino-Header, host-testbar) | `lib/core/` | `protocol` (Parser, Formatierer, Prüfsumme), `safety_fsm`, `pid`, `us_classifier`, `hub_fsm`, `quadrature` (Tabelle), `ramp` |
| HAL | `include/hal.h`, `src/hal_nano_every.cpp` | Pins lesen/schreiben, Zeit, PWM, ADC, NeoPixel, I²C |
| Anwendung | `src/main.cpp`, `src/drive.cpp`, `src/aux.cpp` | Verbindung von Core und HAL, je nach Rolle kompiliert |
Die gesamte Logik liegt in `core` und ist ohne Hardware testbar (Abschnitt 11).

### 3.2 Hauptschleife (kooperativ, kein Betriebssystem)
| Aufgabe | Takt | Rolle |
|---|---|---|
| Serielle Eingabe lesen und parsen | jede Schleife | alle |
| Not-Aus, Bumper, Endschalter einlesen, entprellen | 1 ms | DRIVE/AUX |
| Sicherheitsautomat und Watchdogs auswerten | jede Schleife | DRIVE |
| Ultraschall-Zustandsmaschine | 30 ms-Slots | DRIVE |
| PID-Regler und Rampe | 20 ms | DRIVE |
| Hub-Zustandsmaschine | 10 ms | AUX |
| ADC (ein Kanal je Durchgang, 16-fach Mittelwert) | 100 ms | AUX |
| INA219 lesen | 100 ms | AUX |
| LED-Aktualisierung (nur wenn keine serielle Eingabe wartet, höchstens 10 Hz) | 100 ms | AUX/ALL |
| Telemetrie senden | gemäß 2.3 | je Rolle |
| Hardware-Watchdog füttern | jede Schleife | alle |
Encoder und Ultraschall-Echo laufen in Interrupts (`attachInterrupt`, auf dem Every an allen Pins möglich). Gemeinsame Variablen werden mit kurzer Sperre kopiert (`noInterrupts()` für ≤ 10 µs).

### 3.3 Hardware-Watchdog
Der WDT des ATmega4809 (ca. 1 s) wird in jeder Schleife gefüttert. Bei Hängen setzt er den Chip zurück → Neustart in **BOOT_LOCKED** mit Relais aus. API in der megaAVR-Umgebung beim Anlegen prüfen **[prüfen]**.

### 3.4 Speicherbudget (Schätzung)
RX-Puffer 96 B, TX-Puffer 160 B, NeoPixel 3 × 8 × 3 B = 72 B plus Bibliothek, Stack 512 B, Telemetriezähler ca. 100 B → deutlich unter 2 KB RAM von 6 KB. Flash ca. 15–20 KB. Kein dynamischer Speicher (`malloc`/`String` verboten). [Annahme]

---

## 4. Antrieb (DRIVE)

### 4.1 Encoder
Quadratur-Dekodierung auf beiden Kanälen im Interrupt (Zustandstabelle, 4 Zählungen je Periode). Zähler `int32`. `ENC_INVERT_L/R` korrigiert das Vorzeichen. Die Zahl der Ticks je Radumdrehung ist **nicht** Teil der Firmware (steht in `robot.yaml`, Kalibrierung C1).

### 4.2 Regler
- Takt 20 ms. Rate = Differenz der Ticks / dt, danach gleitender Mittelwert über 2 Messungen.
- Ausgang: `u = KFF·ziel + KP·e + KI·∫e + KD·de/dt` (alle Gewichte als Ganzzahl ×1000, intern in Festkomma), begrenzt auf `PWM_MAX`.
- `PWM_MIN_L/R` (Totzone): Ist |Ziel| > 0, wird |u| mindestens auf diesen Wert angehoben.
- Anti-Windup: Integral wird auf ±`PWM_MAX/KI` begrenzt und eingefroren, solange der Ausgang begrenzt ist.
- Rampe: `ACC` (Ticks/s je Sekunde) begrenzt die Änderung des Solls. Ein Stopp durch Sicherheit (Abschnitt 5) umgeht die Rampe.
- Ziel 0 und |Rate| < `STOP_TPS` (Startwert 5): Motor aus und **Bremse** (siehe 4.3).
- Stillstandserkennung für den Pi: Ziel ≠ 0, aber keine Ticks für 1 s → `$EVT` 52 (Encoder-Fehler), der Pi meldet E112.

### 4.3 L298N-Ansteuerung
Vorwärts: IN1 = HIGH, IN2 = LOW (invertierbar, `MOT_INVERT`), EN = PWM. **Bremse:** IN1 = IN2 = LOW bei EN = HIGH (Kurzschluss der Motoranschlüsse über die unteren Transistoren). Leerlauf: EN = LOW. Der Jumper `5V_EN` der L298N ist bei Motorversorgung 5 V zu entfernen (`bauteile_auswertung.md`).

### 4.4 PWM-Frequenz
Standard-`analogWrite` des Every liegt im hörbaren Bereich (ca. 1 kHz, Pfeifen möglich). Optional Anhebung über den TCA0-Vorteiler, wenn kein Konflikt mit `millis()` (TCB3) entsteht **[prüfen]**. Kein Muss für M1.

---

## 5. Sicherheitslogik (DRIVE)

### 5.1 Zustände und Übergänge (Quelle der Wahrheit, als Testtabelle `firmware/test/vectors/safety_fsm.csv`)
| Aktueller Zustand | Ereignis | Bedingung | Neuer Zustand | Aktion |
|---|---|---|---|---|
| (Start) | Reset/Einschalten | Not-Aus gedrückt | ESTOP | Arming aus, `$HELLO` |
| (Start) | Reset/Einschalten | Not-Aus nicht gedrückt | BOOT_LOCKED | Arming aus, `$HELLO` |
| BOOT_LOCKED | `$ARM,1` | Not-Aus nicht gedrückt | ARMED | Arming an, Sollwerte 0, Fahrbefehle erst nach 200 ms |
| BOOT_LOCKED | `$ARM,1` | Not-Aus gedrückt | BOOT_LOCKED | `$ERR,5` |
| BOOT_LOCKED | Not-Aus gedrückt | – | ESTOP | – |
| ARMED | Not-Aus gedrückt | – | ESTOP | Arming aus, Motoren bremsen, EVT 10 |
| ESTOP | Not-Aus gelöst | – | WAIT_ACK | EVT 11 |
| WAIT_ACK | `$ARM,1` | Not-Aus nicht gedrückt | ARMED | wie oben |
| WAIT_ACK | Not-Aus gedrückt | – | ESTOP | – |
| ARMED | `$ARM,0` | – | WAIT_ACK | Arming aus, Bremse, Flag Software-Stopp |
| ARMED | keine gültige Pi-Nachricht seit `WD_HARD_MS` (5 s) | – | WAIT_ACK | Arming aus, EVT 50 |
Gültige Pi-Nachrichten sind VEL und PING mit korrekter Prüfsumme. Nach `WD_SOFT_MS` (0,5 s) ohne gültige Nachricht: Ziel 0, Bremse, Flag `pi_link_lost`, Zustand bleibt ARMED. Kommt wieder eine gültige Nachricht, wird das Flag gelöscht (EVT 51).

### 5.2 Arming-Ausgang und Relais
- Der Arming-Ausgang speist den Eingang des Sicherheitsrelais (Pololu-Modul). Der Not-Aus-Öffner liegt in der Spulenversorgung (Entwurf `pinplan_nano.md` §8). Der Taster trennt also auch bei hängender Software.
- **Beim Start (erste Anweisungen in `setup()`)**: Arming-Pin auf Ruhepegel. **REQ-ELEC-01** verlangt zusätzlich externe Pull-Widerstände, damit der Pin während Reset und Bootloader nicht das Relais anzieht.
- Die Firmware prüft nach dem Arming nichts über die Relaisrückmeldung (keine Rückleseleitung vorhanden). Die Diagnose läuft über `v_rail` auf der AUX-Rolle (Motorschiene muss nach ARMED Spannung führen, sonst Hinweis „relay or fuse“).

### 5.3 Richtungssperren
| Bedingung | Wirkung |
|---|---|
| Abgrund vorn aktiv (US-Klasse 2 bestätigt) | `block_forward`: positive Translation wird zu 0 begrenzt |
| Abgrund hinten aktiv | `block_backward` |
| Mindestens ein Bumperbit aktiv | `block_forward` |
Die Sperre gilt für die **Summe** der Radsollwerte: Die lineare Komponente `(l + r)/2` wird auf ≥ 0 bzw. ≤ 0 begrenzt, die Drehkomponente `(r − l)/2` bleibt erhalten (Drehen auf der Stelle ist erlaubt). Beim **Auslösen** wird das Ziel sofort auf 0 gesetzt und gebremst (ohne Rampe), danach folgen die Sollwerte der gesperrten Richtung nicht mehr. Die Sperre endet, sobald die Bedingung nach der Entprellung wieder entfällt. Der Pi startet die Wiederanfahr-Strategie (Doc 1 §6.4).

### 5.4 Entprellung
Bumper und Endschalter: 5 ms (`BUMP_DEBOUNCE_MS`). Not-Aus: 5 ms für „gelöst“, **sofort** für „gedrückt“. Ultraschall: Ein = 2 aufeinanderfolgende Abgrund-Messungen, Aus = 3 Boden-Messungen.

---

## 6. Sensorik und Bremsweg-Budget

### 6.1 Ultraschall (Grove Ultrasonic Ranger, ein Signalpin je Sensor)
- Ablauf je Messung (nicht blockierend): Pin als Ausgang, Impuls 10 µs HIGH, Pin als Eingang, Interrupt auf beide Flanken, Echo-Breite aus `micros()`. Zeitlimit 25 ms (350 cm ≈ 20,4 ms). Abstand in mm = Breite in µs × 0,1715 (20 °C) [Wissen].
- **Zeitslots:** 30 ms je Slot, abwechselnd vorn und hinten → jeder Sensor alle 60 ms (ca. 16,7 Hz), keine Übersprechung.
- **Klassifizierung** (Parameter `US_FLOOR_F/R`, `US_CLIFF_MARGIN`, `US_OBST_MARGIN`):
  - kein Echo oder `d > floor + cliff_margin` → **Abgrund** (Klasse 2),
  - `d < floor − obst_margin` → **Hindernis** (Klasse 3),
  - sonst **Boden** (Klasse 1).
  Startwerte: `cliff_margin` 40 mm, `obst_margin` 40 mm, `floor` aus Kalibrierung C6.
- Der Sensor ist **nach unten geneigt** montiert (REQ-MECH-06). Der Pi benötigt für Kartenpunkte die Neigung `us_pitch_deg` und den Bodenpunktabstand `us_ground_x`.
- Gültigkeit: Die Klasse „unbekannt“ gilt bis zu den ersten Messungen und blockiert **nicht**.

### 6.2 Bumper
Drei Endschalter, `INPUT_PULLUP`, aktiv-LOW, Bitmaske 1 links, 2 mitte, 4 rechts. Jede Änderung erzeugt `$EVT,30`. Hinten gibt es keinen Bumper.

### 6.3 Bremsweg- und Latenzbudget (Grundlage für REQ-MECH-06)
| Posten | Wert |
|---|---|
| Slot-Periode je Sensor | 60 ms |
| Bestätigung (2 Messungen) | bis 120 ms nach dem ersten Abgrund-Echo, im ungünstigsten Fall plus 60 ms bis zur ersten Messung = 180 ms |
| Echo- und Verarbeitungszeit | ≤ 30 ms |
| Latenz gesamt (ungünstig) | ≈ 0,21 s |
| Weg bei 0,15 m/s | ≈ 3,2 cm |
| Bremsweg des Roboters (Annahme, zu messen in C6) | ≈ 2 cm |
| **Summe** | ≈ 5 cm |
REQ-MECH-06 verlangt 8 cm Vorlauf: Reserve ≈ 3 cm. Bei 0,20 m/s (manuell) ≈ 4,2 + 3 = 7,2 cm, noch innerhalb. Formel: `v_max ≤ (Vorlauf − Bremsweg) / Latenz`. Ergibt der Test C6 einen größeren Bremsweg, muss die Geschwindigkeitsgrenze sinken oder der Vorlauf wachsen. [Claude, Rechnung]
Der **Bumper** wirkt in < 10 ms (Interrupt-nahe Abfrage mit 1 ms Takt).

---

## 7. Hub (AUX)

- Ansteuerung wie 4.3 (L298N #2), `HUB_PWM_DEFAULT` 200, **Sanftanlauf** von 0 auf Ziel-PWM in 300 ms.
- **Lease:** Läuft nur, solange ein `$HUB` jünger als 500 ms ist (der Pi sendet mit 10 Hz). Fällt der Pi aus, hält der Hub an, der Wagen bleibt hängen (selbsthemmende Winde, REQ-MECH-03).
- Hoch: Stopp bei aktivem Endschalter oben. Ist er schon aktiv: sofort Zustand TOP. Runter analog mit unten. Beide Endschalter aktiv → Fehler 2, Motor aus.
- **Zeitlimit:** `HUB_TIMEOUT_MS` (Startwert 8000, später 1,5 × gemessene Hubzeit, C12). Überschreitung → Motor aus, Fehler 1, `$EVT,42`. Fehler bleibt bis `$CLR`.
- Keine Stromüberwachung (D-26). Die Sicherheit gegen Blockieren besteht aus Zeitlimit und der Trennung durch das Sicherheitsrelais.
- Not-Aus: Das Relais trennt die Hubmotorversorgung hart (gleiche Schiene wie der Antrieb, D-04). Die AUX-Firmware weiß davon nichts und braucht keine Rückleitung.

---

## 8. Anzeige, Signale, Relais (AUX)

### 8.1 NeoPixel
Bibliothek `Adafruit_NeoPixel` (megaAVR unterstützt) [Wissen, bestätigen]. Drei Streifen à 8 LEDs, Standardhelligkeit `LED_BRIGHT` 64 von 255 (Stromgrenze, Abschnitt 12). Muster werden in der Firmware animiert: BLINK 1,5 Hz, LAUFLICHT 80 ms je Schritt. Die Aktualisierung sperrt Interrupts je Streifen ca. 0,25 ms. **Risiko:** In Rolle ALL kann das Zeichen der seriellen Schnittstelle verloren gehen (zwei Bytes Puffer der Hardware) → deshalb Aktualisierung nur bei leerem seriellen Puffer, höchstens 10 Hz, bei Rolle AUX ohne zeitkritische Aufgaben unkritisch (daher Variante A).

### 8.2 Signalplan (Pi-Knoten `status_signals`, Teil von `robotros_mission`)
| Zustand | Links | Rechts | Hinten | Buzzer |
|---|---|---|---|---|
| Stehend, bereit | dauerhaft gedämpftes Blau | dauerhaft gedämpftes Blau | dauerhaft gedämpftes Rot | aus |
| Fährt geradeaus | dauerhaft Weiß | dauerhaft Weiß | dauerhaft Rot | aus |
| Kurve links (ω > 0,2 rad/s) | blinkend Orange | dauerhaft Weiß | Lauflicht Orange links | aus |
| Kurve rechts | dauerhaft Weiß | blinkend Orange | Lauflicht Orange rechts | aus |
| Rückwärts | blinkend Weiß | blinkend Weiß | dauerhaft Weiß | Muster 1 |
| Wartet (Person/Hindernis) | dauerhaft Orange | dauerhaft Orange | dauerhaft Orange | aus |
| Hub aktiv | Lauflicht Grün | Lauflicht Grün | aus | aus |
| Lädt | blinkend Grün (langsam) | blinkend Grün | aus | aus |
| Gesperrt/Not-Aus/Fehler | blinkend Rot | blinkend Rot | blinkend Rot | Muster 3 für 2 s |
Farbwerte Orange (255,110,0), Weiß (180,180,180), Rot (255,0,0), Grün (0,200,0), Blau (0,0,80).

### 8.3 Relais (Pololu 2-Kanal, SPDT)
Kanal 1 Laden (Kontaktplatte → Akku), 2 Power (nur für den optionalen Unterspannungsabschalter, D-27), 3 Reserve. Die Sicherheitsrelais gehört zur Rolle DRIVE (Arming) und ist **nicht** über `$REL` schaltbar. Aktive Pegel im Pin-Header (Abschnitt 1.3). Beim Start sind alle Relais im Ruhepegel.

---

## 9. Spannungs- und Strommessung (AUX)

- **ADC:** `analogReference(INTERNAL4V3)` **explizit** setzen **[geprüft: Arduino-Referenz listet für megaAVR INTERNAL4V3, und die Doku nennt für DEFAULT/INTERNAL 0,55 V, daher nie auf Standardwerte verlassen]**. Nach dem Umschalten die erste Messung verwerfen. 16-fach Mittelwert, ein Kanal je Durchgang. Auflösung bei Teiler 5:1 ≈ 21 mV am Akku.
- Kanäle: `v_batt` (Akku), `v_chg` (Ladekontakt vor dem Relais), `v_rail` (Sicherheitsschiene hinter dem Relais, Diagnose). Der Pi rechnet `V = adc_mv/1000 × 5,0 × gain + offset` (Teiler 30 k/7,5 k [DB]), `gain/offset` aus der Kalibrierung C8.
- **INA219:** `Wire` mit 400 kHz, Bibliothek `Adafruit_INA219`, Kalibrierung „32 V / 2 A“ (Auflösung 0,1 mA, Bereich ±3,2 A mit 0,1 Ω) [Wissen, bestätigen]. Wird er beim Start nicht gefunden: Bit7 in `caps` bleibt 0, `i_ma = −32768`, Pi nutzt die Ladezeitgrenze `charge_max_h_no_ina` (4 h) statt des Stromkriteriums.

---

## 10. Konfigurationsschlüssel (`$CFG`)

Alle Werte Ganzzahlen. Die Bridge sendet **alle** Schlüssel nach jedem `$HELLO` aus `firmware.yaml`. Es gibt **keine** Speicherung im EEPROM (eine Wahrheit auf dem Pi).

| Schlüssel | Rolle | Einheit / Skalierung | Startwert | Kalibrierung |
|---|---|---|---|---|
| `KP`, `KI`, `KD` | DRIVE | ×1000 (PWM je Ticks/s) | 0, 0, 0 (bis C4) | C4 |
| `KFF` | DRIVE | ×1000 (PWM je Ticks/s) | aus C4 | C4 |
| `PWM_MIN_L`, `PWM_MIN_R` | DRIVE | 0–255 | 60 | C5 |
| `PWM_MAX` | DRIVE | 0–255 | 255 (bei 5 V), bei 12 V-Fallback 150 | C10 |
| `ACC` | DRIVE | Ticks/s² | 1500 | C4 |
| `TPS_MAX` | DRIVE | Ticks/s | aus Radumfang und `max_speed` | C2 |
| `STOP_TPS` | DRIVE | Ticks/s | 5 | – |
| `MOT_INV`, `ENC_INV` | DRIVE | Bit0 links, Bit1 rechts | 0 | Bring-up |
| `US_FLOOR_F`, `US_FLOOR_R` | DRIVE | mm | 120 (Platzhalter) | C6 |
| `US_CLIFF_MARGIN`, `US_OBST_MARGIN` | DRIVE | mm | 40, 40 | C6 |
| `US_CONFIRM_ON`, `US_CONFIRM_OFF` | DRIVE | Anzahl | 2, 3 | – |
| `BUMP_DEBOUNCE_MS` | DRIVE/AUX | ms | 5 | – |
| `WD_SOFT_MS`, `WD_HARD_MS` | DRIVE | ms | 500, 5000 | – |
| `HUB_TIMEOUT_MS` | AUX | ms | 8000 | C12 |
| `HUB_PWM` | AUX | 0–255 | 200 | C12 |
| `LED_BRIGHT` | AUX | 0–255 | 64 | – |
| `INA_SHUNT_MOHM` | AUX | mΩ | 100 | – |

---

## 11. Tests

### 11.1 Host-Tests (PlatformIO `native`, Unity)
Ohne Hardware, laufen in CI. Pflichtumfang:
- **Protokoll:** Parser/Formatierer, Prüfsumme, Überlänge, abgebrochene Telegramme, Zusatzfelder. Gemeinsame **Goldene Testvektoren** `firmware/test/vectors/protocol_vectors.json` werden **auch von den Python-Tests der Bridge und von `fake_nano`** genutzt, damit beide Seiten identisch bleiben.
- **Sicherheitsautomat:** jede Zeile der Tabelle 5.1 als CSV-Testfall, plus Watchdog-Übergänge und Richtungssperren (5.3).
- **Regler:** Sprungantwort gegen ein einfaches Motormodell, Anti-Windup, Totzonenanhebung, Rampenbegrenzung.
- **US-Klassifizierung:** Entprellung, kein Echo = Abgrund, Slot-Folge.
- **Hub:** Endschalterlogik, Lease, Zeitlimit, Fehlerzustand.

### 11.2 Hardware-in-the-Loop (Nano Every am Pi, ohne Roboter)
Aufbau mit LEDs/Tastern/Poti als Ersatz: `tools/hil_smoke.py` sendet `WHO`, `CFG`, `ARM`, `VEL`, `LED`, `HUB` und prüft Antworten und Zeiten (Rundlaufzeit `$PING` ≤ 20 ms, Telemetrierate ±10 %, Not-Aus-Taster-Ersatz erzeugt EVT 10 in < 20 ms).

### 11.3 Bring-up-Sketche (`firmware/test_sketches/`)
`blink_pins` (jeden Pin einzeln), `encoder_test`, `us_test` (Messwerte und Zeit), `bumper_test`, `relay_probe` (aktiven Pegel und Reset-Verhalten feststellen, Oszilloskop/Multimeter), `neopixel_test`, `ina219_test`, `adc_test` (Referenz und Teilerfaktor), `serial_loopback` (Durchsatz 115200).

---

## 12. Risiken und offene Punkte der Firmware

| Nr | Risiko | Gegenmaßnahme |
|---|---|---|
| F1 | Bibliotheken für den klassischen Nano laufen auf dem Every nicht 1:1 | Nur getestete Bibliotheken (NeoPixel, INA219, Wire), eigene Encoder-/Ultraschall-Routinen, HIL-Test früh |
| F2 | NeoPixel-Update sperrt Interrupts und kann serielle Bytes verlieren | Rolle AUX (Variante A), Aktualisierung nur bei leerem Puffer, ≤ 10 Hz |
| F3 | Relaislogik und Reset-Verhalten der Pololu-Module unbekannt | `relay_probe`, REQ-ELEC-01, Header-Konstanten |
| F4 | LED-Strom: 24 LEDs weiß bei Helligkeit 100 % ≈ 1,4 A | Standard 25 % (ca. 0,35 A), bei Dauer-Weiß weiter dimmen |
| F5 | Ultraschall am Teppich/Schrägen liefert Fehlalarme | Entprellung 2/3, Sperrzonen als Zweitsicherung, Bodenart im Raum glatt (Runde 18) |
| F6 | Einzelner Ultraschall deckt Seitenkanten nicht ab | Sperrzonen (Doc 1 §8.3), Drehen nur bei Boden-Klassen |
| F7 | Datenblatt des Nano Every fehlt im Projekt | Datenblatt ABX00028 nachladen (nicht blockierend) |
| F8 | PWM-Pfeifen der L298N | optional TCA0-Vorteiler, kein M1-Muss |
