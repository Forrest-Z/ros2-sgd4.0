# Localization

Die Lokalisierung basiert auf der Masterarbeit von Julian Holst. Es kommen ein Partikelfilter und ein Kalmanfilter zum Einsatz.

Wenn GNSS Status = 4, dann nutze GNSS, Odometrie und Magnetometer in einem Kalmanfilter zur Lokalisierung

## State Machine

Implementierung einer State Machine, um je nach Lokalisierungsgenauigkeit unterschiedliche Algorithmen zu nutzern.

## Particle Filter

Wird zur Positionsschätzung eingesetzt

Die Schätzung der Position wird über das Geschwindigkeitsmodell, die GPS-Koordinaten und die UWB-Technologie durchgeführt. Mehrere Messungen werden gleichzeitig aufgenommen und kombiniert. Dies wird wieder über die beschriebene Sensordatenfusion in Kapitel 2.4 bewerkstelligt.

## Kalman Filter

Wird zur Orientierungsschätzung eingesetzt.

Die Schätzung der Ausrichtung wird unter Verwendung der Winkelgeschwindigkeit des Geschwindigkeitsmodells, den Werten des Magnetometers und über die GPS-Koordinaten durchgeführt.



## GPS Status 1 & 2 & 5

GNSS, Odometrie, UWB, Magnetometer, LiDAR Scan Matching zur Lokalisierung


## GPS Status 4

GNSS, Odometrie, UWB (kann in diesem Schritt optimiert werden), Magnetometer (eingeschränkt, wenn LiDAR keine zufriedenstellenden Ergebnisse liefert), LiDAR Scan Matching zur Orientierung, LiDAR Scan zur Hinderniseintragung in die Karte


## Scan Matching

1x pro Sekunde, um die Abweichung des Kompasses zu kompensieren

Möglichkeit 1: Abstand der Punkte zum nächstgelegenen Hindernis aus der Karte -> Abstand minimieren
Möglichkeit 2: Linie aus Punkten generieren -> Winkel zum nächstgelegenen Hindernis berechnen