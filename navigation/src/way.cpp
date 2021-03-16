/*
 * File:   way.cpp
 * Author: Aike Banse
 *
 * Created on January 28, 2021, 11:07 AM
 * 
 * Erstellt eine navigationsfähige Datei in der Nodes mit den nächstliegenden verbundenen Nodes aufgelistet werden mit den dazu gehörigen Referenzen.
 */

#include <fstream>
#include <string>

#include "way.h"

using namespace std;

// Leert die Navigationsdatei, schreibt Datei Anfang und Ende und ruft Funktionen auf
void navigationDatWay(string zielDat, string navigationDat) {
	
	fstream f;

	// Deklarieren und initialisieren von für das Programm wichtiger Variablen
	string nodeArray[100]	= {};
	string docAnfang		= "<nodelist id='1'>";
	string docEnde			= "</nodelist>";

	// Leeren der Navigationsdatei
	f.open(navigationDat, f.out | f.trunc);
	f.close();

	// Schreibt den Anfang in die Navigationsdatei
	f.open(navigationDat, f.app);
	f << docAnfang << "\n";
	f.close();

	// Aufruf der Funktionen
	nodesSuchen(zielDat, nodeArray);						// Sucht die Nodes und speichert diese in einem Array
	for (int k = 0; nodeArray[k] != ""; k++) {				// Ruft die Funktion auf bis nix mehr im Nodearray steht
		nodesSchreiben(zielDat, navigationDat, nodeArray, k);	// 
	}

	// Schreibt das Ende in die Navigationsdatei
	f.open(navigationDat, f.app);
	f << docEnde;
	f.close();
}

// Sucht die Nodes und speichert diese in einem Array
void nodesSuchen(string zielDat, string nodeArray[]) {

	ifstream sourceDat(zielDat);

	// Deklarieren und initialisieren von für das Programm wichtiger Variablen
	int nodenr		= 0;
	int idAnfang	= 12;
	int idEnde		= 0;

	string zeile;
	string ID			= "";
	string idBegrenzung = "'";

	while (getline(sourceDat, zeile)) {				// Wird ausgeführt solange Zeilen in der gefilterten Datei sind
		if (zeile.find("<node") != string::npos) {		// Trifft zu wenn in der aktuellen Zeile "<node" steht
			idEnde = idAnfang;								// Speichert den Wert von "idAnfang" in "idEnde"
			while (zeile[idEnde] != idBegrenzung[0]) {		// Wird ausgeführt solange er nicht die begrenzung der ID findet
				idEnde++;										// Zählt "idEnde" hoch
			}
			ID.resize(idEnde - idAnfang);					// Verändert die Größe von ID auf die Größe der aktuellen ID
			for (int i = 0; i < idEnde - idAnfang; i++) {	// Läuft solange i der ID-Länge entspricht
				ID[i] = zeile[i + idAnfang];					// Speichert jede Stelle der ID in den ID-String
			}
			nodeArray[nodenr] = ID;							// Speichert die ID im Nodearray
			nodenr++;										// Zählt die gefundenen ID´s hoch
		}
	}
}

// 
void nodesSchreiben(string zielDat, string navigationDat, string nodeArray[], int k) {

	fstream f;
	
	ifstream sourceDat(zielDat);
	
	// Deklarieren und initialisieren von für das Programm wichtiger Variablen
	int nodenr = 0;

	string zeile;
	string koords;
	string nodeRefs[10]		= {};
	string nodeAnfang		= "<node id='";
	string koordsAnfang		= "lat='";
	const char koordsEnde	= ' ';
	const char nodeEnde		= '>';

	
	while (getline(sourceDat, zeile)) {									// Durchläuft die Datei Zeile für Zeile
		if (zeile.find("<node") != string::npos) {							// wenn in der Zeile "<node" gefunden wird
			if (zeile.find(nodeArray[k]) != string::npos) {
				for (int i = 0; i <= zeile.length(); i++) {
					if (zeile[i] == koordsAnfang[0] && zeile[i + 1] == koordsAnfang[1] && zeile[i + 2] == koordsAnfang[2] && zeile[i + 3] == koordsAnfang[3] && zeile[i + 4] == koordsAnfang[4]) {
						int b = 22;
						for (int s = b; 1 == 1; s++) {
							if (zeile[i + s] == koordsEnde || zeile[i + s] == nodeEnde) {
								b = s;
								break;
							}
						}
						koords.resize(b);
						for (int a = 0; a <= b; a++) {
							koords[a] = zeile[i];
							i++;
						}
					}
				}
				f.open(navigationDat, f.app);
				f << "  " << nodeAnfang << nodeArray[k] << "' " << koords << nodeEnde << "\n";
				f.close();
				nodeRefFinden(zielDat, nodeRefs, nodeArray[k]);
				int b = 0;
				while (nodeRefs[b] != "") {
					f.open(navigationDat, f.app);
					f << nodeRefs[b] << "\n";
					f.close();
					b++;
				}
				getline(sourceDat, zeile);
				while (zeile.find("<tag") != string::npos) {
					f.open(navigationDat, f.app);
					f << zeile << "\n";
					f.close();
					getline(sourceDat, zeile);
				}
				f.open(navigationDat, f.app);
				f << "  </node>" << "\n";
				f.close();
			}
		}
	}
}

void nodeRefFinden(string zielDat, string nodeRefs[], string aktuelleNode) {

	ifstream sourceDat1(zielDat);

	int refnr = 0;
	int doppelt = 0;

	string aktuellezeile;
	string letzteZeile;
	string ref[2];

	while (getline(sourceDat1, aktuellezeile)) {
		if (aktuellezeile.find("<way") != string::npos) {
			getline(sourceDat1, aktuellezeile);
			while (aktuellezeile.find("<nd ref='") != string::npos) {
				letzteZeile = aktuellezeile;
				getline(sourceDat1, aktuellezeile);
				if (letzteZeile.find(aktuelleNode) != string::npos) {
					ref[0] = aktuellezeile;
					break;
				}
				else if (aktuellezeile.find(aktuelleNode) != string::npos) {
					ref[0] = letzteZeile;
					getline(sourceDat1, aktuellezeile);
					if (aktuellezeile.find("<nd ref='") != string::npos) {
						ref[1] = aktuellezeile;
					}
					break;
				}
			}
		}
		for (int o = 0; o < 2; o++) {
			doppelt = 0;
			if (ref[o] != "") {
				for (int i = 0; i < 10; i++) {
					if (nodeRefs[i] == ref[o]) {
						doppelt = 1;
					}
				}
				if (doppelt != 1) {
					nodeRefs[refnr] = ref[o];
					refnr++;
				}
			}
		}
	}
}