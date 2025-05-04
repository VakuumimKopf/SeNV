# SeNV (Situationsevaluierendes Navigations Vehicle)

### Weitere Informationen stehen im GitHub Wiki

### Aufgabe 1: Fahrbahn folgen

In dieser Aufgabe wird ein Fahrzeug programmiert, das autonom einer Fahrbahn folgt. Dabei werden folgende Anforderungen umgesetzt:

- **Fahrbahn folgen**: Das Fahrzeug bleibt zwischen zwei durchgehenden weißen Linien.
- **Hinderniserkennung und Anhalten**: Erkennt das Fahrzeug ein Hindernis auf der Strecke (mittels Laserscanner), bleibt es in einem Abstand von 10–20 cm davor stehen.
- **Weiterfahren bei freier Strecke**: Sobald das Hindernis nicht mehr detektiert wird, wird die Fahrt automatisch fortgesetzt.
- **Ignorieren seitlicher Hindernisse**: Hindernisse, die sich neben der Strecke befinden, werden ignoriert.
- **Statusausgabe über Topic**: Der aktuelle Status des Fahrzeugs („fahren“ oder „vor Hindernis warten“) wird über ein ROS-Topic ausgegeben – jedoch nur bei einem Wechsel des Status.


### Aufgabe 2: Rote Ampel erkennen

Aufbauend auf Aufgabe 1 wird in dieser Aufgabe die Funktionalität zur Erkennung und Reaktion auf Ampelsignale implementiert.

- **Rote Ampel erkennen**: Das Fahrzeug ist in der Lage, eine rote Ampel zu erkennen.
- **Anhalten bei roter Ampel**: Bei Erkennung einer roten Ampel hält das Fahrzeug an.
- **Weiterfahren bei grüner Ampel**: Sobald die Ampel auf grün wechselt, setzt das Fahrzeug seine Fahrt automatisch fort.
-  **Statusausgabe über Topic**: Der aktuelle Status des Fahrzeugs („fahren“ oder „vor roten Ampel warten“) wird über ein ROS-Topic ausgegeben – jedoch nur bei einem Wechsel des Status.
