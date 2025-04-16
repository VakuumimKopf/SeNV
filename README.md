# SeNV

## Voraussetzungen für ROS 2 Humble

Um dieses Projekt mit ROS 2 Humble auszuführen, stellen Sie sicher, dass die folgenden Pakete installiert sind:

### Erforderliche Pakete
- `ros-humble-desktop` (oder eine Minimalinstallation mit `ros-humble-base` und zusätzliche benötigte Pakete)
- Abhängigkeiten (falls benötigt):
  - `ros-humble-rclpy`
  - `ros-humble-geometry-msgs`
  - `ros-humble-sensor-msgs`
  - `ros-humble-nav-msgs`
  - `ros-humble-tf2` und `ros-humble-tf2-ros`

### Installationsanweisungen

1. **Fügen Sie die ROS 2-Quellen hinzu:**
   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install -y curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

   



Allgemiene Hinweise:
- Kommentare/ Variablen/ klassen etc. im code in englisch, kleingeschrieben, mit Unterstrich verbinden
- Issue/ Pull Request/ Commits etc. deutsch mit engl. Bezug zum Code


Hinweis zu Projects:
Bei Zuweisung einer Aufgabe:
  - Aufgabenstellung steht in infobox des Issues
  - Assigness ausfülle, Startdartum und Deadline angeben
  - Status auf dem Laufenden halten
Neue Issues anlegen:
  - mit Label versehen
  - ggfs. als Subissue eine Aufgabe zuordnen
  - zu Projekt zuordnen, Status angeben

Erster Entwurf für UML Diagramm:
https://www.planttext.com?text=PP3T3SCW38GdO0Slm09H7x99PH6hia9CP9FEN_93GEaBuixdxmIZPTIyBT4vcrb0av5p8Q8PJCfVqge2BgG8LPQ3r0AAjCXAolSTTNw795Mpv9Ada3b9xqSKElM-3I_lQzC0AuVczeLbTiXhoVkjzTtEN-CzQPtd4mz-1HJsxh9ye1y_QJ1SgJvKmOtXt4WobV_z0G00 
