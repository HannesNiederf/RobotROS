# ADR 0001: Projektname robotROS

Status: angenommen (Hannes, 23.11.2026)

## Entscheidung
Das Projekt heißt **robotROS**. In Bezeichnern, Pfaden, Paketnamen und Hostnamen steht kleingeschrieben
`robotros`. Der frühere Arbeitsname wird nicht mehr verwendet, außer in der Tabelle unten, damit man die
älteren Dokumente lesen kann.

## Zuordnung alter zu neuen Bezeichnern
Die Spezifikationsdokumente (`softwarespezifikation.md`, `spez_firmware_protokoll.md`,
`spez_parameter_tests.md`, `spez_aenderung_01_rueckwaerts.md`) verwenden noch die linke Spalte.
Wer sie liest, setzt die rechte Spalte ein.

| bisher | neu |
|---|---|
| ROS-Pakete `patrizio_<name>` (z. B. `patrizio_interfaces`, `patrizio_bridge`, `patrizio_mission`) | `robotros_<name>` |
| systemd-Dienst `patrizio.service` | `robotros.service` |
| Verzeichnisse `/var/lib/patrizio/`, `/opt/patrizio/` | `/var/lib/robotros/`, `/opt/robotros/` |
| Datenbank `patrizio.db` | `robotros.db` |
| Umgebungsvariablen `PATRIZIO_MODE`, `PATRIZIO_DEV` | `ROBOTROS_MODE`, `ROBOTROS_DEV` |
| Hostname des Pi `patrizio`, Adresse `patrizio.local` | `robotros`, `robotros.local` |
| Systembenutzer `patrizio` | `robotros` |
| udev-Namen `/dev/patrizio_nano_drive`, `/dev/patrizio_nano_aux`, Regeldatei `99-patrizio.rules` | `/dev/robotros_nano_drive`, `/dev/robotros_nano_aux`, `99-robotros.rules` |
| Repository-Name `robot-patrizio` in älteren Dokumenten | `RobotROS` (so heißt es auf GitHub) |
| Anzeigename in Texten und UI | robotROS |

Nicht betroffen: Entscheidungsnummern (D-nn), Fehlercodes, Topic-Namen (sie tragen den Namen nicht),
der Rechnername des Surface (`hannes`) und die Präsentation des Teams.

## Folgen
- Das Paket `robotros_interfaces` ist das einzige bisher umgesetzte Paket und wurde in diesem Schritt umbenannt.
- Die Spezifikationsdokumente im claude.ai-Projekt bleiben vorerst unverändert. Beim Kopieren nach
  `docs/spec/` wird der Name in den Kopien ersetzt.
- Der Pi bekommt den Hostnamen `robotros` (Anleitung `anleitung_pi_setup.md` im Projekt, Revision 3).

## Prüfstand
Geprüft: Umbenennung im Repository per Suche über alle Dateien, Schnittstellenprüfung und Tests laufen
(siehe Pull Request). Nicht geprüft: die Hostnamenänderung auf einem echten Pi.
