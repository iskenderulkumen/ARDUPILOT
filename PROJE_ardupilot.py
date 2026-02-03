import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

drone = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Drone baglandi.")

check = False

def mesafe_olcme(konum1, konum2):#Gps koordinatlarını alarak ikisi arasındaki mesafeyi hesaplar
    fark_lat = konum2.lat - konum1.lat
    mesafe_lat = fark_lat * 111132 #111132 yazmamın nedeni enlem başına ortalama metre değeri 
    fark_lon = konum2.lon - konum1.lon
    mesafe_lon = fark_lon * 111132 
    return math.sqrt((mesafe_lat*mesafe_lat) + (mesafe_lon*mesafe_lon))

def yaw_durum(derece):#Dronu her wp sonrası home bakıcak sekılde yaw yapmasını sağlar
    global check
    msg = drone.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, derece, 0, 0, 0, 0, 0, 0)
    drone.send_mavlink(msg)

    while True:#While dongusune almamın nedenı time.sleep den daha dinamik calısması
        mevcut_yaw = drone.heading
        aci_farki = abs(mevcut_yaw - derece)
        if aci_farki <= 5:
            print(f"Yaw ayari tamamlandi: {mevcut_yaw}°")
            break
        time.sleep(0.5)

def aci_hesapla(konum1, konum2):#Ikı nokta arasındaki açıyı hesaplar
    kuzey_farki = konum2.lat - konum1.lat  
    dogu_farki  = konum2.lon - konum1.lon  

    aci_radyan = math.atan2(dogu_farki, kuzey_farki) #Burda atan2 kullanarak gps koordinatlarını radyana ceviriyorum
    
    aci_derece = math.degrees(aci_radyan) #Burdada radyanı dereceye çeviriyorum
    return (aci_derece + 360) % 360 #Olası bir negatif değeri pozitif yapıyorum

def ucus_kontrol():#Basit ucus oncesi kontrol fonksiyonu
    print("PRE-CHECK BAŞLIYOR")
    time.sleep(1)
    
    print(f"Batarya seviyesi: %{drone.battery.level}")
    if drone.battery.level is not None and drone.battery.level < 20:
        print("UYARI: Batarya seviyesi %20 den dusuk")

    print("GPS Fix bekleniyor")
    while drone.gps_0.fix_type < 3:
        print(f"GPS Fix: {drone.gps_0.fix_type}")
        time.sleep(1)
    print("GPS fix tamamlandi")
    time.sleep(1)

    
    global home_konum,son_konum
    home_konum = drone.location.global_relative_frame  #Burda home konumunu kaydediyorum
    son_konum = home_konum
    print(f"Home konumu ayarlandi: Lat: {home_konum.lat}, Lon: {home_konum.lon}, Alt: {home_konum.alt} m")

toplam_yol = 0

ucus_kontrol()

if not drone.is_armable:
    print("Drone su an arm edilemiyor")
    time.sleep(1)

print("Drone arm edilebilir durumda.")
drone.mode = VehicleMode("GUIDED")
print("Mod: GUIDED")

drone.armed = True
while not drone.armed:
    print("Arm bekleniyor")
    time.sleep(1)
print("Drone ARM oldu.")

drone.simple_takeoff(20)
print("Takeoff basladi")
hedef_yukseklik = 20

while True:
    yukseklik = drone.location.global_relative_frame.alt
    print(f"Mevcut Yukseklik: {yukseklik:.1f} m")
    if yukseklik >= hedef_yukseklik*0.95 :
        print("Hedef irtifaya ulasildi.")
        break
    time.sleep(1)


def staging(mevcut_konum, mesafe_metre):
    bilesen = mesafe_metre * 0.707 #45 derecenin sinusunu aldım cunku kuzeydoguya 25 metre gitmemiz istendi
    degisim = bilesen * 0.000009 #Yukarda da acikladigim 111132 metrenin tersi
    
    yeni_lat = mevcut_konum.lat + degisim
    yeni_lon = mevcut_konum.lon + degisim
    
    return LocationGlobalRelative(yeni_lat, yeni_lon, 20)



hedef = staging(drone.location.global_relative_frame, 25)
drone.groundspeed = 4
drone.simple_goto(hedef)



print(f"Batarya: {drone.battery.level}%")
if drone.battery.level < 20:
    print("Batarya seviyesi %20 den dusuk. Ucak kapatiliyor.")
    drone.mode = VehicleMode("RTL")
print("GPS Fix bekleniyor")
while drone.gps_0.fix_type < 3:
    print(f"GPS Fix: {drone.gps_0.fix_type}")
    time.sleep(1)
print("GPS fix tamamlandi")

while True:
    kalan = mesafe_olcme(drone.location.global_relative_frame, LocationGlobalRelative(hedef.lat, hedef.lon, 20))
    print(f"Staging'e kalan mesafe: {kalan:.1f} m")
    if kalan < 6:
        print("Staging noktasina varildi.")
        print(f"Batarya: {drone.battery.level}%")
        if drone.battery.level < 20:
            print("Batarya seviyesi %20 den dusuk. Ucak kapatiliyor.")
            drone.mode = VehicleMode("RTL")
        
        break

    time.sleep(1)
print("Staging Hover.")
staging_noktasi = drone.location.global_relative_frame
time.sleep(5)


def hareket(orijinal_konum, kuzey, guney):#Wp noktalarını olusturuyorum

    degisim_lat = kuzey * 0.000009 #111132 metrenin tersi enlem basına dusen metre degeri
    degisim_lon = guney * 0.000009
    
    yeni_lat = orijinal_konum.lat + degisim_lat
    yeni_lon = orijinal_konum.lon + degisim_lon
    
    return LocationGlobalRelative(yeni_lat, yeni_lon, 20) 


hedef_listesi = [] #Onceki wpleri temizliyorum

p1 = hareket(staging_noktasi, 25, 0)   
p2 = hareket(staging_noktasi, 25, 25)  
p3 = hareket(staging_noktasi, 0, 25)   
p4 = staging_noktasi                       

wp_sayac = 1
hedef_listesi = [p1, p2, p3, p4]

for i in hedef_listesi:
    print(f"WP-{wp_sayac} Gidiliyor")
    drone.groundspeed = 6
    drone.simple_goto(i)

    while True:
        kalan_mesafe = mesafe_olcme(drone.location.global_relative_frame, i)
        if kalan_mesafe < 6:
            print(f"WP-{wp_sayac} noktasina ulasildi.")
            break
        time.sleep(1)
    
    print("3 saniye hover yapiliyor.")
    time.sleep(3)

    suan_konum = drone.location.global_relative_frame

    home_acisi = aci_hesapla(suan_konum, home_konum)
    yaw_durum(home_acisi)

    gidilen_mesafe = mesafe_olcme(son_konum, suan_konum)
    toplam_yol += gidilen_mesafe
    son_konum = suan_konum

    print(f"WP-{wp_sayac} LOG KAYDI ")
    print(f"Lat: {suan_konum.lat:.6f}")
    print(f"Lon: {suan_konum.lon:.6f}")
    print(f"Alt: {suan_konum.alt:.1f} m")
    print(f"Toplam Yol: {toplam_yol:.2f} m")

    wp_sayac += 1

print("\nGorev bitti RTL yapiliyor")
drone.groundspeed = 3
drone.mode = VehicleMode("RTL")
