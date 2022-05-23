# Interaction

Für die einfache Interaktion eines Nutzers mit dem Shared Guide Dog 4.0 wird eine Website zur Verfügung gestellt. Damit diese Website von einem mobilen Endgerät aus erreichbar ist, wird ein Webserver und ein Wifi-Hotspot benötigt. 

Dieses Package benötigt weitere Packages aus dem [Robot Web Tools](http://robotwebtools.org/) Projekt.

## Installation rosbridge_suite

[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

```
sudo apt-get install ros-foxy-rosbridge-server
```

## Befehle zum Starten des Webservers

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```





## Installation Webserver

Als Webserver wird [Apache2](https://httpd.apache.org/) genutzt. Die Installation ist über die offiziellen Ubuntu Pakete möglich:

```
sudo apt install apache2
```

Nach der Installation wird der Webserver automatisch gestartet. Die manuelle Steuerung erfolgt mit den folgenden Befehlen. 

```
sudo service apache2 [start | stop | restart | reload]
```

Informationen zur Konfiguration sind in der offiziellen [Dokumentation](https://httpd.apache.org/docs/2.4/getting-started.html) zu finden.

### Konfiguration für Shared Guide Dog

- Resolve Hostname: Einfügen der Zeile `10.42.0.1 sgd.local` in /etc/hosts -> IP Adresse aus `ip addr` -> nach BROADCAST suchen (nur wenn Hotspot aktiv)

## Einrichtung Hotspot