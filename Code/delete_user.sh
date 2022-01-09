#!/bin/bash
if [[ $# -eq 0 ]] ; then
    echo "Benutzer der gelöscht werden soll angeben"
    read username
else
    username=$1
fi
if id "$username" &>/dev/null; then
   echo "Account $username exists, deleting it"
else
   echo "User does not exists, can not delete it"
   exit 1
fi
dialog --title "Benutzer löschen" \
--backtitle "staernwarte Gersbach" \
--yesno "Wollen sie wirklich den Nutzer $username löschen?" 7 60
resp=$?
case $resp in
  0) sudo pkill -KILL -u $username;;
  1) exit;;
  255) exit;;
esac

dialog --title "Benutzer löschen" \
--backtitle "staernwarte Gersbach" \
--yesno "Sind sie sicher?" 7 60
resp=$?
case $resp in
  0) sudo deluser --remove-home $username;;
  1) exit;;
  255) exit;;
esac
sudo rm -rf /home/$username
