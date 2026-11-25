# ADR 0004: Klartextanzeige „Jetzt / Vorhaben“

Status: angenommen (Claude, 25.11.2026; Wunsch von Hannes, Softwaredetail nach Runde 25)

## Anlass
Hannes möchte sehen, was der Roboter gerade tut und was er vorhat, um ihn besser zu verstehen. Gedacht ist
das zuerst in zwei Terminals und später als Element der Oberfläche.

## Entscheidung
- Neues Paket `robotros_mission` (Spec §3.2: Paket `mission`). Es enthält vorerst nur den Knoten
  `activity_narrator` und das Terminalprogramm `activity_view`. Die Logik liegt ohne ROS in `core/activity.py`.
- Neue Nachricht `robotros_interfaces/ActivityState` auf `/activity` (hier gewählt). Sie enthält je einen
  i18n-Schlüssel mit Argumenten für „Jetzt“ (`activity.*`) und „Vorhaben“ (`plan.*`). Die Texte stehen in
  `web_ui/i18n/{en,de,it}.json` (Spec §13.4), geprüft von `tools/check_i18n.py`.
  Es wird alle 0,5 s gesendet, bei Zustandswechseln sofort.
- Quellen:
  - Sicherheitszustand `/hw/safety`
  - Status und Rückmeldung der Nav2-Aktion `navigate_to_pose` (Restweg, geschätzte Restzeit, Anzahl Recoveries)
  - `/behavior_tree_log`, für die laufenden Knoten Wait, BackUp, ClearCostmap und ComputePathToPose
  - `/odom` (Geschwindigkeit)
  - `/collision_monitor_state`
  - `/plan`; dessen letzter Punkt gilt als Ziel
- Schwellwerte stehen in `robotros_bringup/config/activity.yaml`. Die Profilgeschwindigkeit kommt aus
  `speed_profiles.yaml`.
- Bedienung in der Simulation: `tools/sim_start.sh now` und `tools/sim_start.sh plan`, jeweils in einem
  zusätzlichen Terminal.

## Grenzen
- „Fährt langsam: Engstelle oder Kurve“ ist eine Schlussfolgerung aus der Geschwindigkeit. Den genauen Grund,
  warum der Regler bremst (Kostenregelung oder Kurvenradius), veröffentlicht Nav2 nicht.
- Wenn später `/mission/status` (Spec §9.2, `message_key`) Aufträge meldet, kommen deren Schritte hinzu,
  zum Beispiel „Holt Wagen an Station 3“.
