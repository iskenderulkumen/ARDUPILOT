import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

drone = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Drone baglandi.")

p_katsayisi = 2.15
i_katsayisi = 0.15
d_katsayisi = 0.5
x_toplam_hata = 0.0
y_toplam_hata = 0.
x_onceki_hata = 0.0
y_onceki_hata = 0.0
baslangic_zamani = 0
durum = "TAKEOFF"

def arm_ve_takeoff():
    print("Drone GUIDED moduna geçiliyor")
    drone.mode= VehicleMode("GUIDED")
    while drone.mode.name != "GUIDED":
        print("GUIDED moduna geçiş bekleniyor")
        time.sleep(1)
    print("GUIDED moduna geçildi")

    print("Drone arm edilebilir")
    drone.armed = True
    while drone.armed == False:
        print("Drone arm ediliyor")
        time.sleep(1)
    print("Drone arm edildi.")

    drone.simple_takeoff(20)
    while True:
        print("Yükseklik: ", drone.location.global_relative_frame.alt)
        if drone.location.global_relative_frame.alt >= 19:
            print("İstenilen yüksekliğe ulaşildi")
            break
        time.sleep(1)
    print("Merkeze alınarak sekiz çizme görevi başlatılıyor")
    durum = "MERKEZE_AL"
    
def metreye_cevir(x,lat_mi=True):
    if lat_mi:
        return x * 111132
    else:
        return x * 111132 * math.cos(math.radians(drone.location.global_relative_frame.lat))


def pid_x(hata):
    global x_toplam_hata, x_onceki_hata, baslangic_zamani
    suan = time.time()
    zaman_farki = suan - baslangic_zamani
    x_toplam_hata += (hata * zaman_farki)
    p = p_katsayisi * hata
    i = i_katsayisi * x_toplam_hata
    d = d_katsayisi * (hata - x_onceki_hata) / zaman_farki 
    x_onceki_hata = hata
    return p + i + d 

def pid_y(hata):
    global y_toplam_hata, y_onceki_hata, baslangic_zamani
    suan = time.time()
    zaman_farki = suan - baslangic_zamani
    y_toplam_hata += hata * zaman_farki
    p = p_katsayisi * hata
    i = i_katsayisi * y_toplam_hata
    d = d_katsayisi * (hata - y_onceki_hata) / zaman_farki 
    y_onceki_hata = hata
    return p + i + d 


def ned_hiz(velocity_x, velocity_y, velocity_z,yaw):
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000101111000111,
        0, 0, 0,  
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, yaw, 0) 
    drone.send_mavlink(msg)



arm_ve_takeoff()

merkez_konum = drone.location.global_relative_frame
print(f"Merkez Konum: Lat: {merkez_konum.lat}, Lon: {merkez_konum.lon}")

boyut = 15
hiz = 0.5
hiz_limit = 5
tur_sayisi = 2
baslangic_zamani = time.time()

print("Sekiz çizme görevi başliyor")
durum = "SEKIZ_CIZME"

while durum != "BITIS":
    if durum == "SEKIZ_CIZME":
        t = time.time() - baslangic_zamani
        x_referans = boyut * math.sin(hiz * t) 
        y_referans = boyut * math.sin(hiz * t) * math.cos(hiz * t)

        suan_konum = drone.location.global_relative_frame

        x_gercek_mesafe = metreye_cevir(suan_konum.lat - merkez_konum.lat)
        y_gercek_mesafe = metreye_cevir(suan_konum.lon - merkez_konum.lon)

        x_hata = x_referans - x_gercek_mesafe
        y_hata = y_referans - y_gercek_mesafe

        x_hiz = pid_x(x_hata)
        y_hiz = pid_y(y_hata)

        if x_hiz > hiz_limit: x_hiz = hiz_limit
        if x_hiz < -hiz_limit: x_hiz = -hiz_limit
        if y_hiz > hiz_limit: y_hiz = hiz_limit
        if y_hiz < -hiz_limit: y_hiz = -hiz_limit

        drone_yon = drone.attitude.yaw
        ileri_hiz = x_hiz * math.cos(drone_yon) + y_hiz * math.sin(drone_yon)
        saga_hiz = -x_hiz * math.sin(drone_yon) + y_hiz * math.cos(drone_yon)

        ned_hiz(ileri_hiz, saga_hiz, 0, drone_yon)

        toplam_mesafe_suresi = (2 * math.pi / hiz) * tur_sayisi
        toplam_hata = math.sqrt(x_hata**2 + y_hata**2)


        if (t > toplam_mesafe_suresi) and (toplam_hata < 0.5):
            durum = "BEKLEME"
            print("Referans nokta yakalandı ve süre doldu. Görev Bitti.")
            

    elif durum == "BEKLEME":
        ned_hiz(0, 0, 0, drone_yon)
        print("Sekiz çizme görevi tamamlandi RTL modu basliyor ")
        time.sleep(5)
        drone.mode = VehicleMode("RTL")
        while drone.mode.name != "RTL":
            print("RTL moduna geçiş bekleniyor")
            time.sleep(1)
        durum = "BITIS"
    time.sleep(0.1)
print("Program Sonlandi.")
