# ADR 0010: Von Hand fahren und Stationen mit Richtung einlernen

Status: angenommen (Claude, 27.11.2026). Anlass: Test von Hannes am 27.11. – zum Einlernen fehlte eine Möglichkeit,
den Roboter von Hand zu fahren und auszurichten; „Auftrag pausiert“ war auf der Karte nicht erklärt.

## Inhalt
- **Fahrfeld auf der Kartenseite** (Knopf „Fahren“ oben rechts): Bildschirm-Joystick, Umschalter langsam/normal,
  Anzeige von x, y und Richtung (Grad, 0° = nach rechts auf der Karte, gegen den Uhrzeigersinn) mit Pfeil, Knopf
  „Station hier einlernen“ mit Dialog (Name, Art, Wagen vorhanden). Gespeichert werden Position **und Richtung**
  des Roboters.
- Der Joystick sendet `geometry_msgs/Twist` auf `/cmd_vel_manual` mit 10 Hz, solange er gehalten wird, und beim
  Loslassen einmal null. `twist_mux` hat für diesen Eingang 0,5 s Timeout: bricht der Browser ab, hält der Roboter.
  Der Weg führt wie jede Fahrt durch Collision Monitor und Sicherheitsautomat; nichts davon wird umgangen.
- Geschwindigkeiten nach `speed_profiles.yaml`: MANUAL_SLOW (0,10 m/s, 0,4 rad/s, Voreinstellung) und MANUAL
  (0,20 m/s, 0,8 rad/s). Der Test `web_ui/test/joystick.test.mjs` prüft, dass die Werte in der Oberfläche mit der
  YAML-Datei übereinstimmen. Kennlinie quadratisch, 12 % Totzone (hier festgelegt).
- **Leiste „Ein Auftrag ist pausiert“** auf der Karte mit „Fortsetzen“ und „Auftrag abbrechen“, solange die Mission
  in AWAIT_RESUME steht.

## Grenzen
- Nudge-Tasten (D-38) und die eigene Seite „Handbetrieb“ mit Moduswechsel MANUAL (`SetMode`) folgen.
- Die Tastatursteuerung von `tools/sim_start.sh` (teleop_twist_keyboard) begrenzt die Geschwindigkeit nicht auf die
  Profile (Voreinstellung 0,5 m/s und 1,0 rad/s).

## Prüfung
`web_ui/test/joystick.test.mjs`; `tools/web_ui_check.py` (CI): Fahrfeld öffnen, Joystick nach vorn halten → Roboter
fährt, loslassen → Roboter steht, Station über den Dialog einlernen → Station mit der Richtung des Roboters in
`/web/stations`.
