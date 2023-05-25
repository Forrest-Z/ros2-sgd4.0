# SGD Costmap

Die Umgebungsrepräsentation für den Shared Guide Dog ist über zwei Costmaps.

## Map Server

- Lädt und speichert map von Dateisystem


## Global Costmap

- Grundlage für den Global Planner

Wird aktuell nicht genutzt.

## Local Costmap

- Basiert auf der Karte des Map Servers und Sensordaten
- "Ausschnitt" aus der OSM Karte
- Grundlage für Wegplanung im Nahbereich
- Kann sowohl statische wie auch dynamische Hindernisse enthalten
-> Layered Costmap für dynamische/statische Hindernisse?


## Static Vector Layer Costmap Plugin


### Parameters

This node uses the costmap 2D parameters and has no specific parameters.

| Parameter Name | Type           | Default Value  | Description    |
| -------------- | -------------- | -------------- | -------------- |
| log_dir        | String         | ".ros/log/"    | PLOG log directory |
| log_severity   | String         | "I"            | PLOG logging severity |


## Lidar Costmap Plugin

**Warum ein neues Plugin wenn es doch schon das Obstacle Plugin von Nav2 gibt?**

1. Mehr Flexibilität beim Anpassen der Algorithmen
2. Perspektivisch sollen Hindernisse, die vom Lidar aufgenommen wurden, in der (oder einer) Vektorkarte gespeichert werden. Das ist mit dem Obstacle Plugin nicht möglich.
3. Kobination mit AMCL Algorithmus möglich, sodass bei nicht ausreichend genauer Lokalisierung der Scan mitverwendet werden kann und dann keine Hindernisse in die Karte übertragen werden.
4. Tracking von dynamischen Hindernissen wird ermöglicht