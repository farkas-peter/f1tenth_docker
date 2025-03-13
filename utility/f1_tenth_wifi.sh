#!/bin/sh -e

export CONNECTION_NAME='f1tenth'
export CONNECTION_SSID='f1tenth'
export CONNECTION_PASS='f1tenth_2025'
export CONNECTION_IP4='10.0.10.1/24'

nmcli connection add type wifi ifname wlan0 con-name $CONNECTION_NAME autoconnect yes ssid  $CONNECTION_SSID
nmcli connection modify $CONNECTION_NAME 802-11-wireless.mode ap 802-11-wireless.band bg
nmcli connection modify $CONNECTION_NAME wifi-sec.key-mgmt wpa-psk
nmcli connection modify $CONNECTION_NAME wifi-sec.psk $CONNECTION_PASS
nmcli connection modify $CONNECTION_NAME ipv4.addresses $CONNECTION_IP4
nmcli connection modify $CONNECTION_NAME ipv4.method  shared
nmcli connection up $CONNECTION_NAME
