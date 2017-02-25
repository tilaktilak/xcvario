
close all;
clear all;
clc;

data = csvread('log4.csv');
time = data(:,1);
alt = data(:,2);
smoothed_alt = data(:,3);
rate = data(:,4);

plot(time,alt,'b');
hold on;
plot(time,smoothed_alt,'r');
figure
plot(time,rate);
%figure

fe = 125 % fréquence d'échantillonnage
temps = 1; % temps total de mesure en seconde
t=(0:1/fe:temps-1/fe);

% Calcul des vecteur d'états à estimer (le vecteur d'état est normalement l'inconnue du système !)
vecteur_etat = zeros(temps*fe, 2); % [va ; a; b]
vecteur_etat(:,1)=alt(1:temps*fe);
rate_plot = rate(1:temps*fe);
alt_plot = smoothed_alt(1:temps*fe);
alt_plot2 = alt(1:temps*fe);

vecteur_etat(:,2)=zeros(temps*fe,1);
%vecteur_etat(:, 1) = pi * 2*pi*1*sin(2*pi*1*t(:));
%vecteur_etat(:, 2) = -1*pi * cos(2*pi*1*t(:)) + pi;
%vecteur_etat(:, 3) = 10*t(:)+20*sin(2*pi*0.1*t(:));

% Bruit des capteurs (écart type)
bruit_capteur = zeros(2);
bruit_capteur(1) = 0.6; %+/- 0.6m
bruit_capteur(2) = 1;

% Calcul des paramètres mesurés
mesure = zeros(temps*fe, 2); %[va; a]
mesure(:, 1) = alt(1:temps*fe);%vecteur_etat(:, 1)+vecteur_etat(:, 3) + randn(temps*fe, 1)*bruit_capteur(1);
mesure(:, 2) = zeros(temps*fe,1);%vecteur_etat(:, 2) + randn(temps*fe, 1)*bruit_capteur(2);

%[b, a] = butter(1, 5/25, 'low');
%freqz(b, a, 512, 50);
%mesure(:, 1) = filter(b, a, mesure(:, 1));
%mesure(:, 2) = filter(b, a, mesure(:, 2)); 

% Initialisation de Kalman
H = [1 0; 0 0];
R = [bruit_capteur(1)^2 0; 0 bruit_capteur(2)^2];
A = [1 1/fe ;0 1];
Q = eye(2) * 1;
Q(1, 1) = 0.005;
Q(2, 2) = 1;

X = zeros(2, 1);
X(1) = alt(1);%2*pi*1;
X(2) = 0;
X_svg = zeros(temps*fe, 2);
X_svg(1, 1) = X(1);
X_svg(1, 2) = X(2);
P = zeros(2,2);
P = [0 0;0 0];
%P(1, 1) = 0;
%P(2, 2) = 0;
%P(1, 2) = 0;
%P(2, 1) = P(1, 2);
P_svg = zeros(temps*fe, 2);
P_svg(1, 1) = P(1, 1);
P_svg(1, 2) = P(2, 2);

% Calculs
for run = 2 : temps*fe
    % Prédiction
    Xp = A*X;
    Pp = A*P*A'+Q;
    
    % Mise à jour    
    K = (Pp*H')*inv(R+H*Pp*H');
    P = Pp - K*H*Pp;
    X = Xp + K*(mesure(run, :)'-H*Xp);
    
    X_svg(run, 1) = X(1);
    X_svg(run, 2) = X(2);
end

% Affichage
figure_handle=figure(1);clf;

subplot(1, 3, 1);hold on;
plot(t,mesure(:,1),'r+');
plot(t,vecteur_etat(:,1),'k');
plot(t,X_svg(:,1),'g');
legend('Mesures gyroscope','Vitesse angulaire','Estimation');
axis square;
xlabel('temps');ylabel('vitesse angulaire');
set(figure_handle,'name',' Mesures générées');

subplot(1, 3, 2);hold on;
plot(t,mesure(:,2),'r+');
plot(t,vecteur_etat(:,2),'k');
plot(t,X_svg(:,2),'g');
legend('Mesures accéléromètre', 'Angle','Estimation');
axis square;
xlabel('temps');ylabel('angle');
set(figure_handle,'name',' Mesures générées');
figure
plot(t,rate_plot,'r');
hold on;
plot(t,X_svg(:,2),'b');

figure
plot(t,alt_plot,'r');
hold on
plot(t,alt_plot2,'g');
plot(t,X_svg(:,1),'b');
% subplot(1, 3, 3);hold on;
% plot(t,vecteur_etat(:,3),'k');
% plot(t,X_svg(:,3),'g');
% legend('Biais gyroscope','Estimation');
% axis square;
% xlabel('temps');ylabel('biais gyroscope');
% set(figure_handle,'name',' Mesures générées');
