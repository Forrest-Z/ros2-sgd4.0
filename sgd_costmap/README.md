# SGD Costmap

Die Umgebungsrepräsentation für den Shared Guide Dog ist über zwei Costmaps.

## Map Server

- Lädt und speichert map von Dateisystem


## Global Costmap

- Langzeitspeicher für Hindernisse
- Basiert auf OSM Karte

## Local Costmap

- Basiert auf Hindernissen aus Global Costmap und Sensordaten
- "Ausschnitt" aus der Global Costmap
- Grundlage für Wegplanung im Nahbereich
- Kann sowohl statische wie auch dynamische Hindernisse enthalten
-> Layered Costmap für dynamische/statische Hindernisse?