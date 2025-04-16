# SeNV (Situationsevaluierendes Navigations Vehicle)

Allgemiene Hinweise:

- Kommentare/ Variablen/ klassen etc. im code in englisch, kleingeschrieben, mit Unterstrich verbinden
- Issue/ Pull Request/ Commits etc. deutsch mit engl. Bezug zum Code

Hinweis zu Projects: Bei Zuweisung einer Aufgabe:

- Aufgabenstellung steht in infobox des Issues
- Assigness ausfülle, Startdartum und Deadline angeben
- Status auf dem Laufenden halten Neue Issues anlegen:
- mit Label versehen
- ggfs. als Subissue eine Aufgabe zuordnen
- zu Projekt zuordnen, Status angeben

Erster Entwurf für UML Diagramm: [https://www.planttext.com?text=PP3T3SCW38GdO0Slm09H7x99PH6hia9CP9FEN\_93GEaBuixdxmIZPTIyBT4vcrb0av5p8Q8PJCfVqge2BgG8LPQ3r0AAjCXAolSTTNw795Mpv9Ada3b9xqSKElM-3I\_lQzC0AuVczeLbTiXhoVkjzTtEN-CzQPtd4mz-1HJsxh9ye1y\_QJ1SgJvKmOtXt4WobV\_z0G00](https://www.planttext.com?text=PP3T3SCW38GdO0Slm09H7x99PH6hia9CP9FEN_93GEaBuixdxmIZPTIyBT4vcrb0av5p8Q8PJCfVqge2BgG8LPQ3r0AAjCXAolSTTNw795Mpv9Ada3b9xqSKElM-3I_lQzC0AuVczeLbTiXhoVkjzTtEN-CzQPtd4mz-1HJsxh9ye1y_QJ1SgJvKmOtXt4WobV_z0G00)

### Installation & Setup (ROS2 Humble)

#### Voraussetzungen:

- Ubuntu 22.04
- ROS2 Humble Hawksbill ([https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html))

#### Installation von ROS2 Humble (Desktop-Version empfohlen):

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

#### Arbeitsumgebung einrichten:

```bash
mkdir -p ~/senv_ws/src
cd ~/senv_ws/src
git clone <dieses repository>
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

#### Notwendige ROS2-Pakete:

```bash
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-tf2 ros-humble-tf2-ros ros-humble-laser-geometry
```

Optional (für Simulationen, falls benötigt):

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rviz2
```

#### Launch des Projekts:

Nach dem erfolgreichen Build und dem Sourcen der Umgebung (bei neuer Terminal-Sitzung erneut nötig):

```bash
colcon build
source install/setup.bash
```

kann das Projekt mit folgendem Befehl gestartet werden:

```bash
ros2 launch senv launch_senv.py
```

Dieser Befehl startet alle relevanten Nodes gemäß der Konfiguration in der Datei `launch_senv.py`.

### Aufgabe 1: Fahrbahn folgen

In dieser Aufgabe wird ein Fahrzeug programmiert, das autonom einer Fahrbahn folgt. Dabei werden folgende Anforderungen umgesetzt:

- **Fahrbahn folgen**: Das Fahrzeug bleibt zwischen zwei durchgehenden weißen Linien.
- **Hinderniserkennung und Anhalten**: Erkennt das Fahrzeug ein Hindernis auf der Strecke (mittels Laserscanner), bleibt es in einem Abstand von 10–20 cm davor stehen.
- **Weiterfahren bei freier Strecke**: Sobald das Hindernis nicht mehr detektiert wird, wird die Fahrt automatisch fortgesetzt.
- **Ignorieren seitlicher Hindernisse**: Hindernisse, die sich neben der Strecke befinden, werden ignoriert.
- **Statusausgabe über Topic**: Der aktuelle Status des Fahrzeugs („fahren“ oder „vor Hindernis warten“) wird über ein ROS-Topic ausgegeben – jedoch nur bei einem Wechsel des Status.

