# ADR 0008: Web-Oberfläche Teil 2 (Not-Stopp, Quittieren, Aufträge, Stationen)

Status: angenommen (Claude, 26.11.2026). Baut auf ADR 0005 (Teil 1) und ADR 0007 (`mission_manager` Teil 1) auf.

## Inhalt
- **Kopfzeile:** großer roter Knopf **STOP** rechts (ruft `/mission/software_estop`, §13.2) und eine Anzeige des
  Missionszustands (`/mission/status`). Ohne Verbindung zeigt sie „keine Daten“ (wie ADR 0005/PR #21).
- **Quittierdialog (§6.3):** erscheint, solange der Roboter verbunden und nicht ARMED ist. Ohne gespeicherten
  Auftrag ein Knopf „Roboter freigeben“, mit Auftrag drei Knöpfe (fortsetzen / pausiert lassen / abbrechen) →
  `/mission/ack_safety`. Bei gedrücktem Not-Aus sind die Knöpfe gesperrt.
- **Meldungen:** `/mission/alerts` als Leiste unter den Reitern; Meldungen mit Entscheidung bleiben mit ihren
  Optionen stehen (`/mission/resolve_alert`), andere verschwinden nach 10 s.
- **Reiter Karte / Aufträge / Stationen.**
  - Karte: Antippen → Bestätigen → **GOTO_POINT-Auftrag** über `/mission/create_order` (ersetzt `/goal_pose` aus
    ADR 0005). Stationen erscheinen als Quadrate mit Richtungsstrich (braun = Wagen da, weiß = frei,
    violett = Ladestation).
  - Aufträge: neuer Transport (Quelle nur Abstellplätze mit Wagen, Ziel nur freie), Priorität 2/5/8, Liste mit
    Abbrechen, Wiederholen, Fortsetzen.
  - Stationen: an der aktuellen Roboterposition einlernen (Name, Art, „Wagen vorhanden“), Belegung umschalten
    (`/mission/set_cart_location`), löschen.
- Fehlercodes erscheinen als Text (`error.E###` in `web_ui/i18n/*.json`, alle Codes aus §9.5).
- Reine Hilfsfunktionen in `web_ui/js/mission.js`, getestet mit `node --test` (`web_ui/test/mission.test.mjs`).

## Grenzen
- Nudge-Tasten (D-38), Sperrzonen, Kartenverwaltung, Handbetrieb mit Joystick und Systemseite fehlen noch.
- Die Stationspose ist die Roboterpose beim Einlernen; das Feinjustieren folgt mit dem Docking.
- Kein Bestätigungsdialog vor dem Löschen einer Station (folgt bei Bedarf).

## Prüfung
`tools/web_ui_check.py` (CI, Chromium): Antippen erzeugt einen Auftrag, der Roboter fährt hin, der Auftrag steht
als COMPLETED in der Liste; STOP nimmt die Freigabe weg, der Dialog erscheint, „freigeben“ macht den Roboter
wieder ARMED; Missionszustand IDLE wird angezeigt.
