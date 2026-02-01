import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

drone = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Drone baglandi.")


def mesafe_olcme(konum1, konum2):
    fark_lat = konum2.lat - konum1.lat
    mesafe_lat = fark_lat * 111132
    fark_lon = konum2.lon - konum1.lon
    mesafe_lon = fark_lon * 111132 * math.cos(math.radians(konum1.lat))
    return math.sqrt((mesafe_lat*mesafe_lat) + (mesafe_lon*mesafe_lon))

def yaw_durum(derece):
    msg = drone.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, derece, 0, 1, 0, 0, 0, 0)
    drone.send_mavlink(msg)

def aci_olustur(konum1, konum2):
    lat1 = math.radians(konum1.lat)
    lat2 = math.radians(konum2.lat)
    y_fark = math.radians(konum2.lon - konum1.lon)

    y = math.sin(y_fark) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(y_fark)

    aci = math.atan2(y, x)
    return (math.degrees(aci) + 360) % 360

if not drone.is_armable:
    print("Drone su an arm edilemiyor")
    time.sleep(1)

print("Drone arm edilebilir durumda.")
drone.mode = VehicleMode("GUIDED")
print("Mod: GUIDED")

drone.armed = True
while not drone.armed:
    print("Arm onayi bekleniyor")
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

home_konum = drone.location.global_relative_frame
toplam_yol = 0
son_konum = home_konum


staging_lat = home_konum.lat + (17.68 / 111132)
staging_lon = home_konum.lon + (17.68 / (111132 * math.cos(math.radians(home_konum.lat))))
staging_konum = LocationGlobalRelative(staging_lat, staging_lon, 20)

print("Staging noktasina gidiliyor")
drone.groundspeed = 4
drone.simple_goto(staging_konum)

while True:
    kalan = mesafe_olcme(drone.location.global_relative_frame, staging_konum)
    print(f"Staging'e kalan mesafe: {kalan:.1f} m")
    if kalan < 6:
        print("Staging noktasina varildi.")
        break
    time.sleep(1)

print("Staging Hover.")
time.sleep(5)

hedef_listesi = [
    LocationGlobalRelative(-35.36310168, 149.16543960, 20), 
    LocationGlobalRelative(-35.36274400, 149.16543960, 20), 
    LocationGlobalRelative(-35.36274400, 149.16586300, 20), 
    LocationGlobalRelative(-35.36310168, 149.16586300, 20), 
    LocationGlobalRelative(-35.36310168, 149.16543960, 20)  
]

wp_sayac = 1

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


    home_acisi = aci_olustur(suan_konum, home_konum)
    yaw_durum(home_acisi)
    time.sleep(3) 

    gidilen_mesafe = mesafe_olcme(son_konum, suan_konum)
    toplam_yol += gidilen_mesafe
    son_konum = suan_konum

    print(f"WP-{wp_sayac} LOG KAYDI ")
    print(f"Lat: {suan_konum.lat:.6f}")
    print(f"Lon: {suan_konum.lon:.6f}")
    print(f"Alt: {suan_konum.alt:.1f} m")
    print(f"Toplam Yol: {toplam_yol:.2f} m")

    wp_sayac += 1

print("\nGorev bitti, RTL yapiliyor")
drone.groundspeed = 3
drone.mode = VehicleMode("RTL")
