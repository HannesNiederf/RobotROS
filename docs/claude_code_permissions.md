# Sperrliste für Claude Code (`.claude/settings.json`)

Stand: 24.11.2026 · gilt für jede Claude-Code-Sitzung, die im Ordner des Repositorys startet.

## Zweck
Claude Code läuft auf dem Surface standardmäßig im Auto-Modus: Befehle, die ein Prüfmodell als risikoarm
einstuft, laufen ohne Rückfrage. Sobald ein Nano, Motoren oder Relais angeschlossen sind, darf nichts davon
ohne ausdrückliches OK von Hannes laufen (Regel in `CLAUDE.md`). Die Datei `.claude/settings.json` legt
dafür feste Regeln fest, die Claude Code selbst durchsetzt, unabhängig vom Modell und vom Modus.

Laut Claude-Code-Dokumentation (geprüft am 24.11.2026, Version 2.1.331):
- Regeln werden in der Reihenfolge **deny, ask, allow** geprüft; die erste Übereinstimmung entscheidet.
  Eine allow-Regel kann eine deny- oder ask-Regel nicht aufheben.
- **deny** blockiert in jedem Modus, auch in `bypassPermissions`.
- **ask** fragt auch im Auto-Modus nach.
- Bei zusammengesetzten Befehlen (`&&`, `;`, `|`, Unterbefehle in `$( )`) greift eine deny- oder ask-Regel,
  sobald ein Teilbefehl passt.

## Was gesperrt ist (deny)
| Gruppe | Regeln |
|---|---|
| Administratorrechte | `sudo`, `su`, `pkexec`, `doas` |
| Rohes Schreiben auf Geräte | `dd` |
| Dienste und Rechner | `systemctl start/stop/restart/reload/enable/disable/kill/poweroff/reboot`, `shutdown`, `reboot`, `poweroff` (`systemctl status` bleibt erlaubt) |
| git | erzwungenes Pushen (`--force`, `-f`, `--force-with-lease`), Pushen auf `main` |
| Serielle Geräte | Lesen und Schreiben über Claudes Dateiwerkzeuge und Umleitungen auf `/dev/tty*`, `/dev/serial/**`, `/dev/robotros_*`, `/dev/rplidar` (`/dev/null` bleibt frei) |
| Modus | `bypassPermissions` ist abgeschaltet (`disableBypassPermissionsMode`) |

## Was nur mit Rückfrage läuft (ask)
Claude Code zeigt den Befehl und wartet auf „Ja“. Ein Ja ist das OK für **diesen** Befehl.
| Gruppe | Regeln |
|---|---|
| Firmware aufspielen | `pio run …upload…`, `pio test` mit den Umgebungen `drive`, `aux`, `all` oder ohne Angabe, `avrdude`, `arduino-cli …upload…` |
| Serielle Konsolen | `pio device …`, `pio run …monitor…`, `arduino-cli …monitor…`, `screen`, `minicom`, `picocom`, `socat`, `stty`, `cu`, `miniterm`, `serial.tools` |
| ROS-Befehle, die etwas auslösen | `ros2 topic pub`, `ros2 service call`, `ros2 action send_goal`, `ros2 param set`, `ros2 lifecycle set`, `ros2 run robotros_bridge …`, `ros2 launch robotros_bringup …` |
| Hardware-Werkzeuge aus `spez_parameter_tests.md` | `hil_smoke`, `calib_ticks`, `drive_straight`, `drive_rotate`, `pid_tune`, `deadband`, `battery_profile`, `hub_cycle`, `run_acceptance`, `lidar_align`, `lidar_sector` |
| Zugriff auf andere Rechner | `ssh`, `scp`, `rsync` |

Warum ROS-Befehle fragen: Surface und Pi sind im selben ROS-Netz (`ROS_DOMAIN_ID=42`). Ein
`ros2 topic pub /cmd_vel …` auf dem Surface erreicht also auch den echten Roboter.

`pio test -e native` (Host-Tests ohne Board) fragt nicht. `pio test --environment native` fragt
(die Regel unterscheidet die Schreibweise nicht); dann einfach `-e native` verwenden.

## Grenzen (wichtig)
Die Regeln prüfen den **Befehlstext**, nicht das, was ein Programm intern tut. Laut Dokumentation sind sie
deshalb keine Sicherheitsgrenze:
- Ein Python-Skript, das selbst eine serielle Schnittstelle öffnet, wird von den Dateiregeln nicht erfasst.
- Derselbe Befehl in anderer Form (`/usr/bin/sudo`, `bash -c '…'`) passt nicht auf die Regel.
- Ein `git push` ohne Ziel, während `main` ausgecheckt ist, wird nicht erkannt.

Deshalb gilt zusätzlich:
1. Die Regel in `CLAUDE.md` (nichts an der Hardware ohne OK) bleibt bestehen.
2. Hardware-Skripte bekommen die Namen aus der Tabelle oben, damit die ask-Regeln greifen.
3. Vor dem ersten Hardware-Anschluss wird geprüft, ob die Sandbox von Claude Code auf dem Surface den Zugriff
   auf `/dev/tty*` auf Betriebssystemebene sperren kann (noch nicht untersucht).

## Prüfung
- Ausgeführt (24.11.2026, Claude Code 2.1.331 in der Arbeitsumgebung von Claude): `claude doctor` meldet
  keine Regel, die nicht wie geschrieben wirkt. Eine erste Fassung hatte eine fehlerhafte Regel
  (`git push * main:*`); `claude doctor` hat sie gemeldet, sie ist ersetzt.
- `tools/tests/test_claude_settings.py` prüft Aufbau und Pflichtregeln der Datei.
- **Nicht ausgeführt:** ein echter Lauf, in dem Claude Code einen gesperrten Befehl versucht (in der
  Arbeitsumgebung von Claude ist keine Anmeldung möglich). Das prüft Hannes auf dem Surface.
