# Pin-Plan und Verdrahtungsübersicht (Entwurf, Version 3, Nano Every)

Stand: 18.11.2026 · Status: **Entwurf, Entscheidung zur Pin-Lösung offen** · Gehört zu `claude/projektwissen.md` (Abschnitt 2.7, Auffälligkeit 1)
Quellen: Interview Runden 1–24, `claude/bauteile_auswertung.md` (Einkaufsliste und Datenblätter), `RobotROS presentazione.pdf`, Websuche zum Nano Every (Arduino, Händlerseiten)
Änderungen gegenüber Version 2: **Der Nutzer hat bestätigt, dass er einen Nano Every hat** (Runde 24). Der **INA219 (I²C)** ist als Stromsensor gewählt. Die Motoren laufen **zunächst an 5 V**. Alle Zahlen und Varianten sind auf den Nano Every umgerechnet.
Legende: **[BOM]** · **[DB]** = Datenblatt · **[Nutzer]** · **[Annahme]** · **[Wissen]** = allgemeines Wissen oder Websuche, nicht im Ordner · **[Claude-Vorschlag]**

---

## 1. Entscheidungsstand

- **Pin-Lösung: noch nicht entschieden.** Der Nutzer sagt (Runde 22): „Pins und Schaltplan müssen genau geklärt sein, dann wird entschieden. Im Falle zu vieler Pins kommt entweder ein zweiter Nano oder ein Erweiterungsmodul oder Schieberegister infrage.“ [Nutzer]
- **Entschieden [Nutzer, Runde 24]:** Board = **Nano Every**. Stromsensor = **INA219**. Motorversorgung = **5 V** (bei Problemen wird eine Lösung gesucht).
- **Entschieden [Nutzer, Runde 23]:** Das Sicherheitsrelais trennt Fahr- **und** Hubmotor, Quittierung nach Not-Aus am Surface.

## 2. Nano Every: Eigenschaften [Wissen]

Der Nutzer hat den Nano Every (ABX00028). Ein Datenblatt dazu liegt nicht im Ordner (dort ist der klassische Nano). Nach Arduino und mehreren Händlerseiten:

| Eigenschaft | Wert |
|---|---|
| Prozessor | ATmega4809 (8-Bit megaAVR 0), 20 MHz (Arduino-Core: 16 MHz) |
| Speicher | 48 KB Flash, **6 KB SRAM**, 256 B EEPROM |
| Logikpegel | 5 V, max. 20 mA je Pin |
| Pins | **22 nutzbar: D0–D13 und A0–A7**, A6/A7 auch digital |
| PWM | **5 Pins: D3, D5, D6, D9, D10** |
| Interrupts | **auf allen Pins** |
| I²C | A4 = SDA, A5 = SCL (fest) |
| Seriell | USB über einen separaten Chip (ATSAMD11), **D0/D1 = Serial1 sind frei** (z. B. direkte Verbindung zu einem zweiten Board) |

→ Alles hier ist per Websuche bestätigt, aber **bitte gegen das offizielle Datenblatt (ABX00028) prüfen** und es dem Projekt hinzufügen.

## 3. Bauteilbasis für den Pin-Bedarf

| Bauteil | Anbindung | Quelle |
|---|---|---|
| Fahrantrieb: L298N (Iduino ST1112), 2 Motoren | ENA/ENB (PWM) + IN1–IN4 = 6 Pins | [DB] |
| Hub: zweite L298N, 1 Motor | EN (PWM optional), IN1, IN2 = 2–3 Pins | [BOM] [Nutzer R17] |
| Encoder (DG01D-E), 2 Stück | je H1/H2 (Quadratur), 3,3 oder 5 V = 4 Pins | [DB] |
| Ultraschall Grove Ranger V2.0, 2 Stück (vorne, hinten) | **1 Signalpin je Sensor** = 2 Pins | [DB] [Nutzer R18] |
| Bumper: 3 Endschalter mit langem Hebel (links, mitte, rechts) | je 1 Pin gegen GND = 3 Pins | [BOM] [Nutzer R18] |
| Endschalter Hub (oben, unten) | je 1 Pin gegen GND = 2 Pins | [BOM] |
| Not-Aus (DFRobot FIT0156) | Zustand lesen = 1 Pin | [BOM] [Annahme] |
| Relais: Pololu 2-Kanal, 2 Module mit 5 V + 1 Modul mit 12 V = 6 Kanäle | laut PDF: Laden, Sicherheit, Power ON/OFF, Reserve = 4 Pins. **Das Sicherheitsrelais braucht einen Ausgang** (Quittierung) | [BOM] [PDF] [Nutzer R23] |
| NeoPixel Stick 8 LEDs, 3 Stück (2 seitlich, 1 hinten) | in Reihe 1 Pin, oder 3 getrennte Pins | [BOM] [DB] |
| Aktiver Grove-Buzzer | 1 Pin | [BOM] |
| Spannungssensoren 25 V (Akku, Ladegerät) | je 1 Analogpin = 2 | [DB] |
| **Stromsensor INA219** [Nutzer R24] | I²C (A4/A5), **kein Analogpin** | [BOM] [DB] |
| IMU BNO055 | in v1 nicht eingesetzt [Nutzer], bestellt [BOM]. Später am selben I²C-Bus (Adresse 0x28, INA219 liegt auf 0x40) | [DB] |

**Annahmen, die noch zu prüfen sind:** Ansteuerlogik der Pololu-Relais (aktiv-HIGH oder aktiv-LOW, Datenblatt fehlt im Ordner), Not-Aus als Öffner, Hall-Ausgänge der Encoder (Pull-ups nötig?), Einbauort und Shunt des INA219.

## 4. Pin-Bedarf

| Gruppe | Signale | digital | analog / I²C |
|---|---|---|---|
| Fahrantrieb | ENA, ENB, IN1–IN4 | 6 | |
| Hub | EN, IN1, IN2 | 3 (min. 2) | |
| Encoder | links H1/H2, rechts H1/H2 | 4 | |
| Ultraschall (Grove) | 1 Signalpin je Sensor | 2 | |
| Bumper | links, mitte, rechts | 3 | |
| Endschalter Hub | oben, unten | 2 | |
| Not-Aus (lesen) | 1 | 1 | |
| Relais | Laden, **Sicherheit (Arming)**, Power, Reserve | 4 (min. 2) | |
| NeoPixel | Daten (1 Kette) | 1 | |
| Buzzer | 1 | 1 | |
| Spannungen | V Akku, V Ladegerät | | 2 analog |
| Strom (INA219) | SDA, SCL | | 2 (I²C, A4/A5) |
| **Summe** | | **27 (min. 24)** | **2 analog + 2 I²C** |

**Rechnung:** Der Nano Every hat 22 Pins. Davon gehen 2 für I²C (A4/A5) und 2 für die Analogwerte ab, bleiben **18 für digitale Signale**. Bedarf 27 (min. 24) → **Mangel 9 (min. 6)**. PWM: Fahr-EN ×2 und Hub-EN = 3 von 5 PWM-Pins, das reicht.
→ **Ein einzelner Nano Every reicht nicht.** (Mit dem klassischen Nano wäre der Mangel 11 bzw. 8 gewesen.)

## 5. Einsparungen ohne Zusatzhardware

- Hub-EN fest auf 5 V (Jumper): 1 Pin, Hub dann ohne PWM-Geschwindigkeit.
- Relais „Reserve“ und „Power ON/OFF“ nicht belegen: 2 Pins (wenn der Power-Taster nicht über den Nano läuft).
- Bumper über Widerstandsleiter an einem Analogpin: 2 digitale Pins, kostet aber einen Analogpin.
- 3 NeoPixel-Sticks in einer Kette (Kabel zwischen Seiten und hinten): 1 Pin statt 3.

Mit allem zusammen bleiben **24 Pins Bedarf, Mangel 6**. Es braucht deshalb eine der drei Varianten.

## 6. Varianten

### Variante A: Zweiter Nano Every (zweite USB-Verbindung zum Pi)

**Aufteilung [Claude-Vorschlag]:**
- **Board A „Fahren und Sicherheit“:** Fahrantrieb, Encoder, Ultraschall, Bumper, Not-Aus lesen, **Sicherheitsrelais (Arming)**. 17 von 22 Pins, **5 frei**.
- **Board B „Hub, Laden, Anzeige, Energie“:** Hubmotor, Endschalter, Relais Laden/Power/Reserve, NeoPixel, Buzzer, Spannungssensoren, INA219 (später BNO055). 14 von 22 Pins, **8 frei**.

**Board A:**

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| D2 | Encoder links H1 | D12 | Ultraschall vorne (SIG) |
| D3 | ENA (PWM) | D13 | Ultraschall hinten (SIG) |
| D4 | Encoder links H2 | A0 | Bumper links |
| D5 | ENB (PWM) | A1 | Bumper mitte |
| D6 | IN1 | A2 | Bumper rechts |
| D7 | Encoder rechts H1 | A3 | Not-Aus lesen |
| D8 | Encoder rechts H2 | A4 | Relais Sicherheit (Arming) |
| D9 | IN2 | A5, A6, A7 | frei |
| D10 | IN3 | D0, D1 | Serial1: Verbindung zu Board B (optional) |
| D11 | IN4 | | |

**Board B:**

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| D2 | Hub IN1 | A0 | V Akku |
| D3 | Hub EN (PWM) | A1 | V Ladegerät |
| D4 | Hub IN2 | A4 | SDA (INA219, später BNO055) |
| D5 | Endschalter oben | A5 | SCL |
| D6 | Endschalter unten | D12, D13, A2, A3, A6, A7 | frei |
| D7 | NeoPixel-Daten | D0, D1 | Serial1: Verbindung zu Board A (optional) |
| D8 | Buzzer | | |
| D9 | Relais Laden | | |
| D10 | Relais Power | | |
| D11 | Relais Reserve | | |

**Vorteile:** Fahrregler und Sicherheitslogik laufen getrennt von LED, Relais und Hub (keine NeoPixel-Unterbrechungen, keine I²C-Abhängigkeit). Viele freie Pins. Die Firmware ist für beide Boards derselbe Kern mit anderer Konfiguration. Direkte Verbindung A↔B über Serial1 (z. B. „Halt“-Meldung) ohne Pi möglich.
**Nachteile:** Zweites USB-Gerät am Pi (feste Gerätenamen per Seriennummer oder USB-Port einrichten). Zwei Bridge-Instanzen. Zusätzlicher Strombedarf und Platz.
**Zusatzteile:** 1 weiterer Nano Every (13,11 € laut Einkaufsliste), 1 USB-Kabel, optional 2 Leitungen für Serial1.

### Variante B: Erweiterungsmodul (I²C-Portexpander, z. B. MCP23017, 16 Ein-/Ausgänge)

- **Direkt am Nano Every (zeitkritisch):** Fahrantrieb 6, Encoder 4, Ultraschall 2, Bumper 3, Hub-EN 1, NeoPixel 1, **Sicherheitsrelais (Arming) 1**, Analog 2, I²C 2 = **22 von 22, 0 frei** (D0/D1 werden als normale Pins genutzt).
- **Am Expander:** Hub IN1/IN2 2, Endschalter 2, Not-Aus lesen 1, Relais Laden/Power/Reserve 3, Buzzer 1 = 9 von 16, **7 frei**.

**Vorteile:** Ein einzelnes Board, kein zweites USB-Gerät. Bumper, Ultraschall und Encoder bleiben direkt am Nano. I²C-Bus wird sowieso für INA219 und IMU gebraucht.
**Nachteile:** Alle nativen Pins belegt. Die Ausgänge des Expanders **behalten ihren letzten Zustand, wenn der Nano hängt** (kein selbsttätiger Abfall). Deshalb liegt das Sicherheitsrelais am Nano selbst, und für die anderen Relais braucht es ein Reset-Signal vom Nano an den Expander (kostet 1 Pin am Expander-Reset, ggf. nicht frei). Ein blockierter I²C-Bus blockiert auch INA219 und Endschalter (Timeout nötig). NeoPixel-Updates sperren Interrupts für ca. 0,7 ms [Wissen], bei den DG01D-Encodern (ca. 72 U/s Motorwelle bei 4,5 V) erst ab etwa 10 Impulsen je Umdrehung kritisch [Rechnung, Impulszahl unbekannt].
**Zusatzteile:** 1 Expander-Modul (wenige Euro).

### Variante C: Schieberegister (74HC595 für Ausgänge, 74HC165 für Eingänge)

- **74HC595 × 2 (16 Ausgänge):** Fahrrichtung IN1–IN4 4, Hub IN1/IN2 2, Relais Laden/Power/Reserve 3, Buzzer 1 = 10 Ausgänge, **6 frei**.
- **74HC165 × 1 (8 Eingänge):** Bumper 3, Endschalter 2, Not-Aus 1 = 6 Eingänge, **2 frei**.
- **Direkt am Nano Every:** ENA/ENB/Hub-EN 3 (PWM), Encoder 4, Ultraschall 2, NeoPixel 1, Schieberegister-Leitungen 5 (DATA, CLK, LATCH, LOAD, DATA-IN, mit gemeinsamem Takt), **Sicherheitsrelais (Arming) 1**, Analog 2, I²C 2 = **20 von 22, 2 frei**.

**Vorteile:** Sehr billig, keine Bus-Abhängigkeit.
**Nachteile:** Bumper und Endschalter müssen **abgefragt werden** (kein Interrupt), im 1-kHz-Takt ausreichend. Schieberegister-Ausgänge sind nach dem Einschalten undefiniert (OE-Pin mit Pull-up) und behalten ihren Zustand, wenn der Nano hängt. Fahrrichtung über Schieberegister: Ein Softwarefehler kann Richtungen verdrehen. Mehr Verkabelung.
**Zusatzteile:** 2 × 74HC595, 1 × 74HC165, Entkoppel-Kondensatoren, Pull-ups.

### Nicht genannte Option: GPIO des Pi 5
Relais, Buzzer und LEDs am Pi. Wirkt nur, solange der Pi läuft (Zustände bei Absturz undefiniert). Für sicherheitsnahe Funktionen **nicht empfohlen**. [Annahme]

## 7. Vergleich

| Kriterium | A: Zweiter Nano Every | B: Expander | C: Schieberegister |
|---|---|---|---|
| Freie Pins | A: 5, B: 8 | Nano: 0, Expander: 7 | Nano: 2, 595: 6, 165: 2 |
| Zeitkritisches (Encoder, Ultraschall, Bumper) entlastet | ja (eigenes Board) | teilweise (bleibt am Nano) | teilweise (Bumper per Abfrage) |
| Ausgänge fallen bei Nano-Hänger ab | ja (beide Boards) | nein (Expander hält Zustand) | nein (Register halten Zustand) |
| NeoPixel stört Encoder | nein | gering | gering |
| Neue Software | zweite Instanz | I²C-Treiber | Schieberegister-Treiber |
| Kosten (Größenordnung) | ca. 13 € | wenige Euro | wenige Euro |
| Verdrahtung | wenig | mittel | viel |
| Hauptrisiko | USB-Zuordnung | I²C-Hänger | Startzustände, Abfrage |

**Empfehlung [Claude-Vorschlag]:** **Variante A (zweiter Nano Every)**: die sauberste Trennung, alle Ausgänge fallen bei einem Hänger selbsttätig ab, und die Kosten sind gering. **Variante B ist mit dem Every erstmals komfortabel**, wenn ein zweites USB-Gerät vermieden werden soll. **Die Entscheidung trifft der Nutzer, nachdem Pins und Schaltplan geklärt sind.**

## 8. Sicherheitsrelais mit Quittierung (aus Runde 23)

Der Nutzer will: **Not-Aus löst das Sicherheitsrelais aus, nach dem Lösen des Tasters ist eine Quittierung am Surface nötig.** Der **Hubmotor läuft am selben Relais wie der Fahrantrieb.** [Nutzer]

Umsetzung [Annahme, Entwurf]:
- Die Spule des Sicherheitsrelais wird über den **Öffner des Not-Aus-Tasters** versorgt. Wird der Taster gedrückt, fällt das Relais **hart** ab (ohne Software).
- Zusätzlich schaltet ein **Ausgangspin des Nano (Arming)** die Spule. Nach dem Lösen des Tasters bleibt das Relais aus, bis der Nano nach der **Quittierung am Surface** wieder einschaltet.
- Fehlersicher: Fällt der Nano oder der Pi aus, fällt das Relais ab, **sofern der Ausgang bei Reset nicht schaltet**. Das hängt von der Ansteuerlogik der Pololu-Relais ab (siehe offene Punkte).
- Das Relais trennt die Versorgung der **beiden L298N** (Fahrantrieb und Hub). Die Logikversorgung der L298N bleibt separat (Jumper 5V_EN entfernen, siehe `claude/bauteile_auswertung.md`).
- **Mechanik:** Bei Not-Aus mit angehobenem Wagen steht der Hubmotor stromlos. Die Winde muss den Wagen **selbsthemmend halten**, sonst sinkt der Wagen ab. [Annahme]
- **Nach einem Pi-Neustart** wird das Relais erst nach Bestätigung durch den Bediener geschaltet. [Annahme zur Einheitlichkeit mit Runde 16]

## 9. Verdrahtungsübersicht (Text-Blockdiagramm, vorläufig)

```
12 V Akku (Multipower MP2.4-12C)  ── Sicherung 12 V (zu ergänzen, fehlt in der Liste)
 ├─ Power-Taster (rastend, 16 mm) / PiEnergy-Mini → Pi 5 (5,1 V, max. 3 A)   [BOM] [DB]
 ├─ LB07 Step-Down → 5 V (no safety)  (fehlt in der Liste)                    [PDF]
 │    ├─ Nano Every (ggf. zweites Board), Sensoren (Encoder, Ultraschall, Spannungssensoren, INA219), NeoPixel (Helligkeit begrenzen)
 │    └─ Relaisspulen (5-V-Module)
 ├─ Sicherheitsrelais (Spule über Not-Aus-Öffner + Nano-Arming)               [Nutzer R23]
 │    └─ „safety“-Schiene 5 V → L298N Fahrantrieb + L298N Hub                 [Nutzer R23, R24]
 │         (über L298N bleiben ca. 3 V am Motor, Motoren 3–9 V; Fallback 12 V mit PWM-Begrenzung bei Problemen)
 └─ Ladepfad: Ladestation (13,8 V, 0,7 A) → Bodenplatte → Kontakte am Roboter
      → Relais LADEN → INA219 → Akku   (Spannungssensor „Ladegerät erkannt“, INA219 „Ladeende“)

Pi 5 ──USB──> Nano Every A (Fahren, Sicherheit)  ──> L298N Fahr, Encoder, Ultraschall, Bumper, Not-Aus lesen, Relais Sicherheit
Pi 5 ──USB──> Nano Every B (Hub, Laden, Anzeige) ──> L298N Hub, Endschalter, Relais, NeoPixel, Buzzer, V-Sensoren, INA219 (I²C), später BNO055
Pi 5 ──USB──> RPLidar A1 (USB-Adapter prüfen; Startstrom bis 600 mA)
Board A ──(Serial1, optional)── Board B
Masse: alle Massen gemeinsam sternförmig verbinden.
```
(Bei Variante B oder C entfällt Board B, die Signale wandern an Expander bzw. Schieberegister.)

## 10. Offene Punkte für den Schaltplan

1. **Pin-Lösung:** zweiter Nano Every, Erweiterungsmodul oder Schieberegister (Entscheidung des Nutzers nach Klärung der Pins).
2. **Datenblatt Nano Every (ABX00028)** zum Projekt hinzufügen und die Pin-Angaben (Abschnitt 2) prüfen.
3. **Motorspannung:** 5 V ist der Plan [Nutzer]. Verdrahtung so auslegen, dass später auf 12 V mit PWM-Begrenzung umgesteckt werden kann. Jumper 5V_EN der L298N entfernen. Früh Lasttest. **Kriterium für „Probleme“** (z. B. Mindestgeschwindigkeit, Anfahren mit Wagen und 1 kg) festlegen.
4. **Ansteuerlogik der Pololu-Relais** (aktiv-HIGH oder -LOW) und Verhalten beim Reset des Nano, Kontaktbelastbarkeit.
5. **Wofür wird das 12-V-Relaismodul genutzt** (z. B. Sicherheitsrelais direkt aus 12 V)?
6. **5-V-Leistungsbudget des LB07** (Nano(s), Relaisspulen, Sensoren, NeoPixel, ggf. Motoren). Der LB07 fehlt in der Liste und im Datenblattordner.
7. **Sicherung(en)** und Verpolschutz.
8. **Power-Taster (rastend)**: Anschluss an Akku, PiEnergy-Pads oder Relais.
9. **Encoder:** Polzahl, Pull-ups, Hall-Versorgung 3,3 V oder 5 V.
10. **INA219:** Einbauort (Ladepfad oder Akku), Shunt (0,1 Ω?) und damit Messbereich (±3,2 A reicht für den Ladestrom, nicht für Motorspitzen).
11. **Verhalten beim Start:** Alle Ausgänge (Relais, Motorrichtungen) müssen nach Einschalten oder Reset in einem sicheren Zustand sein.
12. **NeoPixel:** Eine Kette (3 Sticks in Reihe, Kabel zwischen den Seiten und hinten) oder 3 getrennte Datenpins?
13. **Bei zwei Boards:** USB-Zuordnung (feste Gerätenamen) und ggf. Serial1-Verbindung.
