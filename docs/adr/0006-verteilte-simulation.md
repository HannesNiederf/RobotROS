# ADR 0006: Verteilte Simulation (Gazebo auf dem Surface, Robotersoftware auf dem Pi)

Status: angenommen (Claude, 25.11.2026; Wunsch von Hannes, Option B vom 24.11.2026)

## Inhalt
- `robotros_sim/launch/robot_software.launch.py` enthält alles, was im echten Betrieb auf dem Pi liefe:
  `sim_hw_adapter` (anstelle von `nano_bridge` und den Nanos), Navigation (`mode` mapping|nav), `sim_helper`
  und Web-Oberfläche. `sim.launch.py` bindet es weiter ein (`robot_software:=true`, Standard, alles auf einem
  Rechner). Mit `robot_software:=false` startet es nur Gazebo, Robotermodell und RViz.
- `tools/pi_robot_software.sh MODE [MAP]` läuft auf dem Pi. Es startet die Robotersoftware im Bereich 77 und gibt
  alle 5 s die Last als „PILOAD cpu_busy=…% mem_used=…MiB“ aus. Bei Verbindungsende hält es sauber an.
- `tools/sim_start.sh mapping|nav --pi[=HOST]` (Standard `robot-ros@robotros.local`): Auf dem Surface laufen
  Gazebo und RViz, die Robotersoftware wird per SSH auf dem Pi gestartet. Im Modus `nav` wird die Karte vorher
  nach `~/robotros_sim_maps/` kopiert. Am Ende erscheint eine Zusammenfassung der Pi-Last. Die Oberfläche kommt
  vom Pi (`http://robotros.local:8080`).
- Die DDS-Erkennung läuft in dieser Betriebsart über das WLAN (`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`), weiterhin
  nur im Bereich 77, nie im Bereich 42 des Roboters. `now`/`plan`/`rtf`/`save` in weiteren Terminals übernehmen das
  automatisch, solange eine `--pi`-Sitzung läuft (Markierungsdatei `build/sim_start/.pi_session`).
- Voraussetzungen: SSH-Schlüssel (`ssh-copy-id`) und derselbe Stand des Workspace auf dem Pi, gebaut.
  Eine Anleitung dazu folgt in `anleitung_pi_setup.md` Teil H.

## Grenzen
- Im Betrieb nach Spezifikation (D-20) spricht DDS nicht über WLAN; das gilt nur für diese Simulationsart.
- `/clock` und `/scan` gehen über das WLAN. Netzlast und Verzögerung über den Handy-Hotspot: siehe Nachtrag;
  die Ursache der Verzögerung im ersten Pi-Lauf ist noch nicht geklärt.
- CI prüft die Aufteilung mit `--pi=local`: beide Teile laufen auf einem Rechner, aber als getrennte Starts. Damit
  ist sichergestellt, dass nichts fehlt; das Netz selbst prüft das nicht.

## Nachtrag 25.11.2026: erster Lauf mit dem echten Pi
Ergebnis (Hannes, `mapping --pi`): Pi-Last im Mittel 54 %, höchstens 86 %, Speicher 705 MiB. Der Pi schafft die
Rechenarbeit. Der Roboter reagierte aber stark verzögert und ruckartig; die Karte zeigte graue Keile (Scans mit
falscher Lage).

Gemessen in der CI (ein Rechner, Echtzeitfaktor ca. 0,7, `tools/sim_start.sh rates`): zusammen ca. 120 KiB/s und
ca. 450 Nachrichten/s, am häufigsten `/joint_states` und `/clock` (je ca. 114/s) und `/tf` (73/s). Die Datenmenge
ist für ein WLAN klein; die Ursache ist damit **nicht geklärt**. Vermutungen, die der nächste Pi-Lauf prüfen soll:
1. **Gemischte Middleware:** Ubuntus `~/.bashrc` endet in nicht interaktiven Shells vor den angehängten Zeilen. Der
   SSH-Befehl lief daher vermutlich mit Fast DDS (Vorgabe von Jazzy), das Surface mit Cyclone DDS.
2. **Multicast im WLAN** (Hotspot, Energiesparen des Pi-WLAN) verzögert Daten.
3. Viele kleine Nachrichten (`/clock`, `/tf`).

Änderungen:
- `sim_start.sh` und `pi_robot_software.sh` setzen `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` selbst (änderbar mit
  `SIM_RMW`). Der Pi meldet Middleware und WLAN-Energiesparen (`iw … get power_save`) zu Beginn; `sim_start.sh`
  zeigt beides am Ende zusammen mit der Last.
- In `--pi`-Sitzungen gilt `robotros_sim/config/cyclonedds_wifi.xml`: Multicast nur für die Erkennung, Daten per
  Unicast.
- Gazebo-Uhr auf `/sim/clock`; `clock_relay.py` (Surface) gibt sie höchstens `clock_relay_hz` (100) Mal pro
  simulierter Sekunde auf `/clock` weiter. Der schnellste Zeitgeber mit Simulationszeit hat 100 Hz
  (`step_period_s`).
- Radwinkel (`/joint_states`) mit 20 Hz statt bei jedem Physikschritt; sie bewegen nur die Räder in der Anzeige.
- `sim_start.sh --pi=HOST` misst während der Sitzung jede Sekunde die Antwortzeit zum Pi (`ping`) und zeigt am Ende
  Verlust und Laufzeiten.
