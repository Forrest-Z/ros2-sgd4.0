# Hardware Driver

In diesem Unterordner sind alle Hardware Treiber, die für den Betrieb des Shared Guide Dog benötigt werden, enthalten.

## Capacitive Touch

Sensormodell: `Eigenbau`

Der Capacitive Touch Sensor ist an den Handgriffen des Shared Guide Dog befestigt und erkennt, ob der Nutzer/die Nutzerin die Handgriffe festhält. Ist dies nicht der Fall, wird der Shared Guide Dog gestoppt.

## GPS

Sensormodell: `Navilock NL-602U u-blox 6`
[Produktwebsite](https://www.navilock.de/produkte/S_61840/merkmale.html)

Der GPS Sensor liefert die Position anhand des GPS Signals. Die Position wird im lokalen und im globalen WGS84 Koordinatensystem ausgegeben. 

## IMU

Sensormodell: `Bosch BNO055`
[Produktwebsite](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)

Die IMU liefert eine absolute Ausrichtung des Shared Guide Dog und die Beschleunigung und Drehrate.

## Laser 1D

Sensormodell: `Pololu VL53L1X`
[Produktwebsite](https://www.pololu.com/product/3415)

Der 1D Lasersensor ist zwischen den Handgriffen befestigt und überwacht die Distanz zwischen dem Shared Guide Dog und dem Nutzer. Wird der Abstand zu groß, verringert sich die Geschwindigkeit, wird der Abstand kleiner, erhöht sich die Geschwindigkeit.

## Lidar

Sensormodell: `Sick TiM571-2050101`
[Produktwebsite](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim5xx/tim571-2050101/p/p412444)

Der 2D Lidarsensor überwacht die Region vor und neben dem Shared Guide Dog. Er wird für die Hinderniserkennung und Lokalisierung eingesetzt.

## Motortreiber


