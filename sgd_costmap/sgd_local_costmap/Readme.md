# SGD Local Costmap

Stichworte:
- Obstruction Map
- Polygon Map

**Aktuelle Fragestellungen:**
- Ist ein Matching von Linien zu Linien (Hindernis zu Hindernis) sinvoll oder ein Matching von aktuellen Scanpunkten zu Linien (-> Line based SLAM)?
- Wie können zwei Linien miteinander verrechnet werden?

**TODOs:**

## Obstacle Detection / Costmap creation

Ziel: Objekte werden aus den Scan-Daten (im Ausblick auch aus weiteren Daten) generiert und sind in der Karte verfolgbar und klassifizierbar.

Vorgehen:
- Das Array aus Scan-Werten wird in Subsets geteilt. Anschließend kann auf diesen Subsets ein Split-And-Merge Algorithm (Douglas and Peucker) angewendet werden.
- Sind bereits Hindernisse in diesem Gebiet der Karte bekannt, findet ein Matching statt

### Teilung in Subsets

Wenn der Abstand zwischen zwei aufeinanderfolgenden Scan-Punkten größer als ein Grenzwert (zurzeit 1.0m) ist, wird das Array an dieser Stelle getrennt und ein neues Subset erzeugt.

Bevor der Split-And-Merge Algorithmus angewendet werden kann, werden die Punkte in ein x-y-Koordinatensystem transformiert.

$$ x = \cos(\varphi)\cdot d $$
$$ y = \cos(\varphi)\cdot d $$

Mit $d$ = gemessene Distanz und $\varphi$ = aktueller Winkel des Punktes.



### Split-And-Merge Algorithm

Der Split-And-Merge-Algorithmus basiert auf Ausgleichsgeraden, die am weitesten entfernten Punkt geteilt und neu berechnet werden. Der Algorithmus terminiert, wenn der am weitesten entfernte Punkt näher als ein Grenzwert ist. Der Grenzwert definiert wie fein Strukturen nachgebildet werden können, erhöht jedoch auch exponentiell(??) die Berechnungsdauer.

**Berechnung der Ausgleichsgerade**

$$ m = \frac{\sum_{i=1}^{n}\left(x_i-\bar{x}\right)\left(y_i-\bar{y}\right)}{\sum_{i=1}^{n}\left(x_i-\bar{x}\right)^2} $$

mit $n$ = Größe des Subsets

**Datenpunkt mit dem maximalen Abstand zu Linie**

Achtung: Hier wird nicht der maximale Abstand zur Ausgleichsgerade, sondern der maximale Abstand zu der Gerade, die den ersten und letzten Punkt aus dem Subset verbindet, berechnet.

$$ d_i = \frac{\left|\left(x_n-x_0\right)\left(y_0-y_i\right)-\left(x_0-x_i\right)\left(y_e-y_0\right)\right|}{\sqrt{\left(x_n-x_0\right)^2+\left(y_n-y_0\right)^2}} $$

mit $i \in [0,1,...,n] $ und $n$ = Größe des Subsets

Für den am weitesten entfernten Messpunkt wird der Index gespeichert, das aktuelle Subset an dem Index getrennt und der Split-And-Merge Algorithmus mit den zwei neuen Subsets erneut durchgeführt.

**Schnittpunkt berechnen**

Wurden aus dem Subset mehr als eine Linie generiert, dann werden beide Linien verbunden, indem der Schnittpunkt berechnet wird.

Aus den Gleichungen für die beiden Geraden
$$ l_1: \begin{pmatrix}x_{11} \\ y_{11}\end{pmatrix} + t\cdot \begin{pmatrix}x_{11}-x_{12} \\ y_{11}-y_{12}\end{pmatrix} $$
und
$$ l_2: \begin{pmatrix}x_{21} \\ y_{21}\end{pmatrix} + u\cdot \begin{pmatrix}x_{21}-x_{22} \\ y_{21}-y_{22}\end{pmatrix} $$

folgt $u$ zur Berechnung des Schnittpunkts:

$$ u = \frac{x_{11}\left(y_{21}-y_{12}\right)+x_{12}\left(y_{11}-y_{21}\right)+x_{21}\left(y_{12}-y_{11}\right)}{x_{11}\left(y_{21}-y_{22}\right)+x_{12}\left(y_{22}-y_{21}\right)+x_{21}\left(y_{12}-y_{11}\right)+x_{22}\left(y_{11}-y_{12}\right)} $$

Durch Einsetzen in Gleichung $l_2$ lässt sich der Schnittpunkt berechnen.

Der Algorithmus endet mit der Rückgabe eines Arrays mit den berechneten Punkten.

## Hindernis-Klasse

Erkannte Hindernisse werden objektorientiert gespeichert und verarbeitet.

Hindernisse werden klassifiziert in *Polygon* und *Circle*. Ein *Polygon*-Hindernis setzt sich aus mehreren geraden Linien zusammen. Ein *Circle*-Hindernis ist eine Spezialisierung des Polygon Hindernisses und bildet eine runde Struktur ab. Hindernisse, die sich aus runden, wie auch anderen Strukuren zusammen setzen, werden als Polygon repräsentiert.

Hindernisse werden im *map*-Koordinatensystem gespeichert, da sich damit statische und dynamische Hindernisse besser verarbeiten lassen. Dazu werden die Koordinaten der Punkte in das *map*-Koordinatensystem transformiert:

$$ x_{map} = \cos(\omega)\cdot x_i-\sin(y)\cdot y_i + x_{robo} \\
y_{map} = \cos(\omega)\cdot y_i+\sin(y)\cdot x_i + y_{robo} $$

$x_{robo}$ bzw. $y_{robo}$ ist die Position des Shared Guide Dog im map-Koordinatensystem. $\omega$ ist die Drehung um die z-Achse.

**Zuordnung des Scans zu Hindernissen**

Damit Hindernisse verfolgt und statische Hindernisse mit einer größeren Sicherheit in die Karte eingetragen werden können, muss eine Zuordnung von erkannten Hindernissen zu bereits früher erkannten Hindernissen erfolgen.

Die Zuordnung von Scan-Punkten zu bereits bekannten Hindernisse erfolgt über den Abstand der Punkte zum Hindernis.

1. Berechne erwarteten Scan auf Basis der bekannten Hindernisse
2. Zuordnung von Scan-Punkten zu bekannten Hindernissen
3. Aus den verbleibenden Punkten werden neue Hindernisse generiert

### Expected Scan

Je Hindernis Berechnung der sichtbaren Punkte:
1. Umwandlung der Hindernisse in ein Polarkoordinatensystem
2. Minimales und maximales phi finden
3. Ausgehend vom minimalen phi Steigungen zu den Nachbarpunkten berechnen. Der Punkt mit der größten Steigung ist der nächste Punkt
4. Alle Punkte in dieser Richtung zur Liste hinzufügen bis Punkt mit maximalem phi erreicht ist

Zusammenführen aller sichtbaren Punkte:
1. Alle Punkte in eine Liste einfügen und nach phi sortieren

### Zuordnung von Scan Punkten







### Ab hier alter Teil

**Zuordnung zweier Hindernisse zueinander**

Damit Hindernisse verfolgt und statische Hindernisse mit einer größeren Sicherheit in die Karte eingetragen werden können, muss eine Zuordnung von erkannten Hindernissen zu bereits früher erkannten Hindernissen erfolgen.

Für die Zuordnung von zwei Hindernissen wird ein Übereinstimmungsmaß eingeführt. Das Maß basiert auf den Steigungen der Linien.

Zur Bestimmung werden die folgenden Schritte durchgeführt:

1. Bestimme Steigung aus Punkten
2. Bestimme modifizierten Sinus aus den Steigungen
3. Übereinstimmungsmatrix durch Division
4. Finde Diagonale mit hoher Übereinstimmung

Das folgende Beispiel verdeutlicht die Rechenschritte.

**Schritt 1:** Bestimme Steigung aus den Punkten

Aus früheren Scans ist bereits ein Hindernis mit den folgenden Koordinaten bekannt:

$$ O_o: \begin{pmatrix}
            -9 & -3 \\
            -6 & -4 \\
            -1.6 & 1.4 \\
            -4 & 3.5 \\
            -6 & 1.5 \end{pmatrix} $$

Aus dem aktuellen Scan wird ein neues Hindernis generiert:

$$ O_n: \begin{pmatrix}
            -9 & -6.5 \\
            -2 & 0.5 \\
            -4.5 & 3.2 \end{pmatrix} $$

Die Position der beiden Hindernisse liegt damit wie in der Abbildung dargestellt.

**TODO:** Abbildung einfügen

Die Steigung zwischen den einzelnen Punkten wird anschließend mit

$$ m_x = O\left[i,0\right] - O\left[i-1,0\right] \\
   m_y = O\left[i,1\right] - O\left[i-1,1\right] $$

bestimmt.

**Schritt 2:** Modifizierter Sinus als Richtungsanzeiger

Zum Vergleich der Steigungen wird ein modifizierter Sinus eingeführt. Die Signum-Funktion ist mit $ sgn(0) = 1 $ definiert.

$$ m = sgn(m_x) \cdot \frac{m_y}{\sqrt{m_x^2+m_y^2}} $$

Die folgende Abbildung zeigt den Wertebereich in einem Winkelbereich von $ [\pi, -\pi] $. Der Grund für die Modifizierung ist, dass Winkel, die um 180° versetzt sind, die also die gleiche Gerade beschreiben, den gleichen Richtungsanzeiger bekommen. 

**Schritt 3:** Berechnung der Übereinstimmungsmatrix

Steigungsvektor 1 (Original): $ m_o = \begin{pmatrix} m_{o,1} \\ m_{o,2} \\ \vdots \\ m_{o,i} \end{pmatrix} $

Steigungsvektor 2 (Neues Hindernis): $ m_n = \begin{pmatrix} m_{n,1} \\ m_{n,2} \\ \vdots \\ m_{n,j} \end{pmatrix} $

Die Conformance Matrix mit der Größe $ i \times j $ setzt sich dann wie folgt zusammen:

$$ M_C = \begin{pmatrix}
            a_{1,1} & a_{1,2} & \cdots & a_{1,j} \\
            a_{2,1} & a_{2,2} & \cdots & a_{2,j} \\
            \vdots  & \vdots  & \ddots & \vdots \\
            a_{i,1} & a_{i,2} & \cdots & a_{i,j}
         \end{pmatrix} $$

Mit $ a_{i,j} = \left\{ \begin{array}{ll} 
                            \frac{m_{n,j}}{2m_{o,i}}+0.5 & if \left|m_{n,j}\right|\lt \left|m_{o,i} \right| \\
                            \frac{m_{o,i}}{2m_{n,j}}+0.5 & else
                        \end{array} \right. $



