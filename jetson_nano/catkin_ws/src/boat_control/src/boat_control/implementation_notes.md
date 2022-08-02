# Sailing logic

Grundsaetzlich reicht erstmal folgendes Vorgehen:

# 1. Windrichtung grob schaetzen
- folgende Schritte sind initial wichtig, sobald das Boot segelt, sollte
   die Windrichtung staendig geschaetzt werden
- Kurs zum Wind festlegen (siehe run.py pt()/st())
- Einige Sekunden warten, bis Kurs stabil ist
- Kompasskurs bestimmen (ahrs.yaw)
- aus Kompasskurs und Kurs zum Wind die Windrichtung schaetzen

# 2. Kompasskurs zum Ziel bestimmen
- Kurs von aktueller Position (lat/lon) zu Wegpunkt bestimmen
- bitte Funktionen getCourseToWaypoint und getDistanceToWaypoint
   testen

# 3. Sollwindrichtung schaetzen
- aus geschaetzter Windrichtung und Kurs

# 4. Fallunterscheidung
* Kurs kann mit dem "gleichen Wind" (von BB/StB) erreicht werden --> Kurs
    zum Wind mit pt() oder st() so waehlen, dass Kompasskurs passt
* Kurs kann mit "Wind von der anderen Seite" erreicht werden --> erst
    Wenden oder Halsen, dann Kurs zum Wind so waehlen, dass Kompasskurs
    passt
* Kurs kann nicht erreicht werden (liegt in Richtung Wind) --> den
    Kurs zum Wind auf 45 Grad setzen und Richtung (pt/st) so waehlen,
    dass der Winkel zum gewuenschten Kompasskurs moeglichst klein ist

 In jedem Fall muss auch das Segel entsprechend eingestellt werden und 
ggf. aufgrund der Fehlschaetzung der wahren Windrichtung nachgeregelt 
werden: Abweichung ist die Differenz aus Kompasskurs und Wunschkurs,
geregelt wird Segelstellung und Soll-"scheinbarer Wind".

Die alte Java-Software hat Kursberechnungen einer Karte vorgenommen, daher 
ist der Code nicht 1:1 transferierbar. Ausserdem ist dort der Mast nicht 
drehbar. Aber im Anhang machen die Funktionen in SimpleCoursePlanner in 
etwa das, was oben skizziert ist.

# Sailing theory

## Port and starboard

port = left side of the vessel
starboard = right side of the vessel

## Tacks: Starboard vs Port

starboard tack: the wind comes from starboard (right side of the vessel)
port tack: the wind comes from port (left side of the vessel)