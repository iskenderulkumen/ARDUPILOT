import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt
import json


drone = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Drone baglandi.")

#Gerekli değişkenleri tanımladım
p_katsayisi = 0.5
i_katsayisi = 0.05
d_katsayisi = 0.2
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


def arm_ve_takeoff():#Arm ve takeoff yapmasını sağlayan fonksiyon
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

    
def metreye_cevir(konum1,konum2):#Gps koordinatını metreye ceviriyorum
    x_fark = konum2.lat - konum1.lat
    y_fark = konum2.lon - konum1.lon

    x_mesafe = x_fark *111132
    y_mesafe = y_fark *111132 * math.cos(math.radians(konum1.lat))
    return x_mesafe,y_mesafe


def pid_x(hata,zaman_farki):#X için pid hesaplaması 
    global x_toplam_hata, x_onceki_hata, hiz_limit
    p = p_katsayisi * hata
    x_toplam_hata += hata * zaman_farki
    i = i_katsayisi * x_toplam_hata
    d = d_katsayisi * (hata - x_onceki_hata) / zaman_farki 
    
    x_onceki_hata = hata
    
    toplam_hiz = p + i + d 

    
    return toplam_hiz   

def pid_y(hata, zaman_farki):#Y için pid hesaplaması
    global y_toplam_hata, y_onceki_hata, hiz_limit
    
    p = p_katsayisi * hata
    y_toplam_hata += hata * zaman_farki
    i = i_katsayisi * y_toplam_hata
    d = d_katsayisi * (hata - y_onceki_hata) / zaman_farki 
    
    y_onceki_hata = hata
    
    toplam_hiz = p + i + d

    
    return toplam_hiz


def globalden_body(x_hiz,y_hiz,yaw):#Aldıgımız ned koordinat hızlarını bodye donusturuyorum
    ileri_hiz = math.cos(yaw) * x_hiz + math.sin(yaw) * y_hiz
    saga_hiz = -math.sin(yaw) * x_hiz + math.cos(yaw) * y_hiz
    return ileri_hiz,saga_hiz

def body_hiz(drone,ileri_hiz, saga_hiz, yukari_hiz,):#Bodye donusturdugum değerleri drone gonderiyorum
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111000111,
        0, 0, 0,  
        ileri_hiz, saga_hiz, yukari_hiz, 
        0, 0, 0, 0, 0) 
    drone.send_mavlink(msg)




#Log olusturmak için gerekli değişkenler
x_referans_log , y_referans_log = [],[]
x_gercek_log , y_gercek_log = [], []
x_hata_log, y_hata_log = [],[]
x_hiz_log ,y_hiz_log = [], []



arm_ve_takeoff()

print("Stabilizasyon bekleniyor")

merkez_konum = drone.location.global_relative_frame #Takeoff aldığım noktayı merkez olarak kaydediyorum
print(f"Merkez Konum Alındı: {merkez_konum.lat}, {merkez_konum.lon}")
durum = ["MERKEZE_AL"]

body_hiz(drone, 0, 0, 0) 
time.sleep(5)

baslangic_zamani = time.time()

print("Sekiz çizme görevi başliyor")
durum = "SEKIZ_CIZME"

while durum != "BITIS":
    if durum == "SEKIZ_CIZME":
        t = time.time() - baslangic_zamani
        #8 çizmemizi sağlayan değişkenler
        x_referans = boyut * math.sin(hiz * t) 
        y_referans = boyut * math.sin(hiz * t) * math.cos(hiz * t)

        suan_konum = drone.location.global_relative_frame

        x_gercek_mesafe, y_gercek_mesafe = metreye_cevir(merkez_konum,suan_konum)
        

        x_hata = x_referans - x_gercek_mesafe
        y_hata = y_referans - y_gercek_mesafe

        #X hızını pid ye gore güncelliyorum
        x_hiz = pid_x(x_hata,guncelleme_suresi)
        if x_hiz > hiz_limit: x_hiz = hiz_limit
        elif x_hiz < -hiz_limit : x_hiz = -hiz_limit
        #Y hızını pid ye gore güncelliyorum
        y_hiz = pid_y(y_hata, guncelleme_suresi)
        if y_hiz > hiz_limit:y_hiz = hiz_limit
        elif y_hiz< -hiz_limit: y_hiz = -hiz_limit


        drone_yon = drone.attitude.yaw
        ileri_hiz,saga_hiz = globalden_body(x_hiz,y_hiz,drone_yon)

        body_hiz(drone,ileri_hiz, saga_hiz, 0)

        #Log listelerini güncelliyorum
        x_referans_log.append(x_referans)
        y_referans_log.append(y_referans)
        x_gercek_log.append(x_gercek_mesafe)
        y_gercek_log.append(y_gercek_mesafe)
        x_hiz_log.append(ileri_hiz)
        y_hiz_log.append(saga_hiz)
        x_hata_log.append(x_hata)
        y_hata_log.append(y_hata)

        toplam_mesafe_suresi = (2 * math.pi / hiz) * tur_sayisi

        print(f"X mesafe: {x_gercek_log[-1]:.3f}, Y mesafe: {y_gercek_log[-1]:.3f}")
        print(f"İleri hiz: {x_hiz_log[-1]:.3f}, Sağa hiz: {y_hiz_log[-1]:.3f}")
        print(f"X hata: {x_hata_log[-1]:.3f}, Y hata {y_hata_log[-1]:.3f}")
        print("----------------------------------------------------------------")

        if t > toplam_mesafe_suresi:
            durum = "BEKLEME"
            print("Referans nokta yakalandı ve süre doldu. Görev Bitti.")
            

    elif durum == "BEKLEME":
        body_hiz(drone,0, 0, 0)
        print("Sekiz çizme görevi tamamlandi RTL modu basliyor ")
        time.sleep(5)
        drone.mode = VehicleMode("RTL")
        while drone.mode.name != "RTL":
            print("RTL moduna geçiş bekleniyor")
            time.sleep(1)
        durum = "BITIS"
    time.sleep(guncelleme_suresi)
print("Program Sonlandi.")

log_verileri = {
    "x_referans": x_referans_log,
    "y_referans": y_referans_log,
    "x_gercek": x_gercek_log,
    "y_gercek": y_gercek_log,
    "x_hiz": x_hiz_log,
    "y_hiz": y_hiz_log,
    "x_hata": x_hata_log,
    "y_hata": y_hata_log
}

dosya_adi = "ucus_loglari.json"
with open(dosya_adi, "w") as f:
    json.dump(log_verileri, f,indent=4) 

print(f"Veriler {dosya_adi} dosyasına kaydedildi.")

#Son olarak logları grafık haline getiriyorum
plt.plot(y_referans_log, x_referans_log, 'r--', label='Referans')
plt.plot(y_gercek_log, x_gercek_log, 'b-', label='Gerçek')

plt.title('Referans ve Gercek Mesafe')
plt.legend()
plt.show()
