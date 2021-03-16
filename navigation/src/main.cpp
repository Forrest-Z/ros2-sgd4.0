/*
 * File:   main.cpp
 * Author: Aike Banse
 *
 * Created on January 7, 2021, 11:47 AM
 *
 * Dieses Programm liest eine .osm-Datei ein und filtert diese. Die Datei wird zuerst nach dem Tag "SharedGuideDog" durchsucht und schreibt die gefilterten Inhalte
 * in eine zweite .osm-Datei. Nachfolgend werden in den Nodes und Ways mit dem Tag "SharedGuideDog" die Referenzen zu anderen Nodes rausgefiltert und ebenfalls in die
 * Zieldatei geschrieben. Nachdem nun vollst�ndig gefiltert wurde muss die Datei navigierbar gemacht werden und wird daf�r umgeschrieben. Folgend muss die erstellte 
 * Datei in eine A*-Funktion eingelesen werden und ein Pfad mit der niedrigsten Gesamtsumme erstellt werden, welcher wieder in einer Datei festgehalten wird.
*/

#include <string>
#include <iostream>

#include "navigation/Filtern.hpp"
#include "navigation/way.hpp"
#include "navigation/aStern.hpp"

using namespace std;

// Hauptprogramm
int main() {

    int s = 0;
    //double StartZielKoords[4] = { 0.0, 0.0, 0.0, 0.0 };

    // Einbinden der Dateipfade der Quell-, Ziel- und Navigationsdatei
    string ursprungsDatei           = "C:\\Users\\Aike\\Desktop\\Uni\\Semester 7\\VSCodeProjekte\\SharedGuideDog\\01_Lohmuehlenpark_augmentiert_way.osm";
    string gefiltertDatei           = "C:\\Users\\Aike\\Desktop\\Uni\\Semester 7\\VSCodeProjekte\\SharedGuideDog\\10_FilterErgebnis.osm";
    string navigationsfaehigeDatei  = "C:\\Users\\Aike\\Desktop\\Uni\\Semester 7\\VSCodeProjekte\\SharedGuideDog\\20_NavigationsFaehigeDaten.osm";
    string endDateiOSM              = "C:\\Users\\Aike\\Desktop\\Uni\\Semester 7\\VSCodeProjekte\\SharedGuideDog\\30_EndNavigationOSM.osm";
    string endDateiNavigation       = "C:\\Users\\Aike\\Desktop\\Uni\\Semester 7\\VSCodeProjekte\\SharedGuideDog\\31_EndNavigation.osm";

    cout << "Bitte geben Sie fuer das komplette ausfuehren der Datei eine 0 ein oder 1 um nur die Navigationsdatei zu erstellen:" << "\n";
    cin >> s;

    if (s == 0) {
        filtern(ursprungsDatei, gefiltertDatei);
        navigationDatWay(gefiltertDatei, navigationsfaehigeDatei);
        s = 1;
    }
    if (s == 1) {
        /*cout << "Bitte geben Sie die Start und Ziel Koordinaten ein. Benutzen Sie bitte die folgende Reihenfolge: Startlaenge, Startbreite, Ziellaenge, Zielbreite" << "\n";
        for (int i = 0; i < 4; i++) {
            cin >> StartZielKoords[i];
        }*/
        float StartZielKoords[4] = { 53.555299, 10.0230275, 53.5551091, 10.022093 };

        aStern(navigationsfaehigeDatei, StartZielKoords, gefiltertDatei, endDateiNavigation, endDateiOSM);
    }

    return 0;
}