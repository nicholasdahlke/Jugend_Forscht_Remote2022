#!/bin/bash
echo "Vollen Namen eingeben:"
read vorname nachname
echo "Email eingeben:"
read email
username=`echo "$vorname$nachname" | tr '[:upper:]' '[:lower:]'`
password=`openssl rand -base64 7`
password_hashed=`mkpasswd -m sha-512 $password`
sudo useradd -m -p $password_hashed -s /bin/bash $username
sudo usermod -a -G dialout,tsusers,plugdev,quotalimited $username
sudo cp -r /home/staernwarten_admin/template/. /home/$username
sudo chown -R $username:$username /home/$username
ls -la /home/$username
echo "-------------------------------"
echo "Username is $username"
echo "Password is $password"
echo "-------------------------------"
groups $username
#------------------------------------
echo "Guten Tag Herr/Frau $nachname" >> .mail.txt
echo "Ihr Account für die staernwarte Gersbach wurde erstellt." >> .mail.txt
echo "Sie können sich nun mit einen beliebigen RDP-Client unter der IP-Adresse 10.140.1.17 anmelden" >> .mail.txt
echo "Viele Grüße," >> .mail.txt
echo "staernwarte Gersbach" >> .mail.txt
echo "-------------------------------" >> .mail.txt
echo "Benutzername ist $username" >> .mail.txt
echo "Passwort ist $password" >> .mail.txt
echo "-------------------------------" >> .mail.txt

mail -s "Account Staernwarte Gersbach" $email < .mail.txt
rm .mail.txt
