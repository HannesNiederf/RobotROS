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
- `/clock` aus Gazebo und `/scan` gehen über das WLAN. Netzlast und Verzögerung über den Handy-Hotspot
  sind ungeprüft.
- CI prüft die Aufteilung mit `--pi=local`: beide Teile laufen auf einem Rechner, aber als getrennte Starts. Damit
  ist sichergestellt, dass nichts fehlt; das Netz selbst prüft das nicht.
