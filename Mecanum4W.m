function [R,p,g,a,n_wh] = Mecanum4W()
% MECANUM WHEEL FAHRZEUG MIT 4 RÄDERN
% Reihenfolge der Räder:
% 1) links vorne
% 2) links hinten
% 3) rechts vorne
% 4) rechts hinten

% Radius der Räder [cm]
R = 5;

% Positionen der Räder im Koordinatensystem B des Fahrzeugs [cm]
p = [10   -10      10     -10; 
    24.11 24.11 -24.11 -24.11]; % Vier Räder
% 15+0.5+4.71+2.5 = 22.71
% 15+0.5+6.11+2.5 = 24.11
 
% Ausrichtung der Räder: Gamma beschreibt den Winkel [rad] zwischen x Achse 
% von B und Rollrichtung der Räder (nicht der Rollen!)
g = [0 0 180 180]*pi/180;
%g = [0 180 180 0]*pi/180;

% Ausrichtung der Rollen: Alpha beschreibt den Winkel [rad] zwischen x
% Achse des Radkoordinatensystems W und der Rollrichtung der Rolle
%a = [45 -45 -45 45]*pi/180;
a = [-45 45 45 -45]*pi/180;

% Anzahl Omni/Mecanum Räder
n_wh = size(p,2);

end