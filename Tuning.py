import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt



#TUNING İCİN OLUSTURDUM KATSAYILARLA OYNAYARAK 10 METRE İLERİ GİTMESİNİ İSTEDİM VE KATSAYILARI AYARLAMAYAYA CALISTIM



drone = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Drone baglandi.")

p_katsayisi = 0.7
i_katsayisi = 0.02
d_katsayisi = 0.3
x_toplam_hata = 0.0
y_toplam_hata = 0.0
x_onceki_hata = 0.0
y_onceki_hata = 0.0
baslangic_zamani = 0
guncelleme_suresi = 0.1
boyut = 15
hiz = 0.2
hiz_limit = 5
tur_sayisi = 2
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
    print("Merkeze alınarak sekiz çizme görevi başlatiliyor")
    durum = "MERKEZE_AL"
    
def metreye_cevir(konum1,konum2):
    x_fark = konum2.lat - konum1.lat
    y_fark = konum2.lon - konum1.lon

    x_mesafe = x_fark *111132
    y_mesafe = y_fark *111132 * math.cos(math.radians(konum1.lat))
    return x_mesafe,y_mesafe


def pid_x(hata,zaman_farki):
    global x_toplam_hata, x_onceki_hata, hiz_limit
    p = p_katsayisi * hata
    
    x_toplam_hata += hata * zaman_farki

    if x_toplam_hata > hiz_limit: x_toplam_hata = hiz_limit
    elif x_toplam_hata < -hiz_limit: x_toplam_hata = -hiz_limit
    
    i = i_katsayisi * x_toplam_hata
    
    d = d_katsayisi * (hata - x_onceki_hata) / zaman_farki 
    
    x_onceki_hata = hata
    
    toplam_hiz = p + i + d 
    
    if toplam_hiz > hiz_limit: toplam_hiz = hiz_limit
    elif toplam_hiz < -hiz_limit: toplam_hiz = -hiz_limit
    
    return toplam_hiz   

def pid_y(hata, zaman_farki):
    global y_toplam_hata, y_onceki_hata, hiz_limit
    
    p = p_katsayisi * hata
    
    y_toplam_hata += hata * zaman_farki
    
    if y_toplam_hata > hiz_limit: y_toplam_hata = hiz_limit
    elif y_toplam_hata < -hiz_limit: y_toplam_hata = -hiz_limit
    
    i = i_katsayisi * y_toplam_hata
    
    d = d_katsayisi * (hata - y_onceki_hata) / zaman_farki 
    
    y_onceki_hata = hata
    
    toplam_hiz = p + i + d
    
    if toplam_hiz > hiz_limit: toplam_hiz = hiz_limit
    elif toplam_hiz < -hiz_limit: toplam_hiz = -hiz_limit
    
    return toplam_hiz


def ned_hiz(drone,velocity_x, velocity_y, velocity_z,):
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111000111,
        0, 0, 0,  
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 0, 0) 
    drone.send_mavlink(msg)


def globalden_body(x_hiz,y_hiz,yaw):
    ileri_hiz = math.cos(yaw) * x_hiz + math.sin(yaw) * y_hiz
    saga_hiz = -math.sin(yaw) * x_hiz + math.cos(yaw) * y_hiz
    return ileri_hiz,saga_hiz



x_referans_log , y_referans_log = [],[]
x_gercek_log , y_gercek_log = [], []
zaman_log = []

arm_ve_takeoff()

print("Stabilizasyon bekleniyor")
time.sleep(3) 

merkez_konum = drone.location.global_relative_frame
print(f"Merkez Konum Alındı: {merkez_konum.lat}, {merkez_konum.lon}")

ned_hiz(drone, 0, 0, 0) 
time.sleep(2)

print("Tuning Modu: 10 Metre İleri Git Testi")
durum = "TUNING"
hedef_x_mesafe = 10.0 
hedef_y_mesafe = 0.0

baslangic_zamani = time.time()
onceki_zaman = time.time() 

while durum == "TUNING":
    suan = time.time()
    gercek_dt = suan - onceki_zaman
    onceki_zaman = suan
    gecen_sure = suan- baslangic_zamani
    
    x_referans = hedef_x_mesafe
    y_referans = hedef_y_mesafe
    
    suan_konum = drone.location.global_relative_frame
    x_gercek_mesafe, y_gercek_mesafe = metreye_cevir(merkez_konum, suan_konum)
    
    x_hata = x_referans - x_gercek_mesafe
    y_hata = y_referans - y_gercek_mesafe
    
    x_hiz = pid_x(x_hata, gercek_dt)
    y_hiz = pid_y(y_hata, gercek_dt)
    
    drone_yon = drone.attitude.yaw
    ileri_hiz, saga_hiz = globalden_body(x_hiz, y_hiz, drone_yon)
    ned_hiz(drone, ileri_hiz, saga_hiz, 0)
    
    x_referans_log.append(x_referans)
    x_gercek_log.append(x_gercek_mesafe)
    y_referans_log.append(y_referans)    
    y_gercek_log.append(y_gercek_mesafe)
    zaman_log.append(gecen_sure)

    if (suan - baslangic_zamani) > 20:
        durum = "BITIS"
        ned_hiz(drone, 0, 0, 0)
        break
    
    time.sleep(0.05) 


print("Program Sonlandi.")
print(f"gidilen metre x-{x_gercek_mesafe}")
drone.mode = VehicleMode("RTL")
plt.plot(zaman_log, x_referans_log, 'r--', label='Referans')
plt.plot(zaman_log, x_gercek_log, 'b-', label='Gerçek')

plt.title('PID Tuning: Zaman vs Mesafe')
plt.xlabel('Zaman')
plt.ylabel('Mesafe')
plt.grid(True)
plt.legend()
plt.show()
