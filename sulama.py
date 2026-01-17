# bluetooth_sulama_gui.py
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QPushButton,
    QFileDialog, QMessageBox, QDateEdit, QHBoxLayout, QComboBox
)
from PyQt5.QtCore import QTimer, QDate
import pyqtgraph as pg
import datetime
import sys
import sqlite3
import pandas as pd
import serial
import calendar

PORT_ISMI = "COM5"   


class SulamaArayuz(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Akıllı Sulama Sistemi")
        self.resize(600, 1200)
        self.setStyleSheet("background-color: black; color: white;")
        
        self.flag=0
        self.flag_open=0
        layout = QVBoxLayout()

        # Bugünün tarihi (string olarak)
        self.bugun_str = datetime.date.today().strftime("%Y-%m-%d")

        # Tarih ve ışık etiketi
        self.tarih_label = QLabel("Tarih: ")
        self.isik_label = QLabel("Anlık Işık Değeri: 0.00 %")
        layout.addWidget(self.tarih_label)
        layout.addWidget(self.isik_label)

        # 1) Nem grafiği
        self.graph_nem = pg.PlotWidget(title="Nem Grafiği")
        self._hazirla_grafik(self.graph_nem, "Nem (%)", "Saat")
        layout.addWidget(self.graph_nem)

        # 2) Işık grafiği
        self.graph_isik = pg.PlotWidget(title="Işık Grafiği")
        self._hazirla_grafik(self.graph_isik, "Işık (%)", "Saat")
        layout.addWidget(self.graph_isik)

        # 3) Sıcaklık grafiği
        self.graph_sicaklik = pg.PlotWidget(title="Sıcaklık Grafiği")
        self._hazirla_grafik(self.graph_sicaklik, "Sıcaklık (°C)", "Saat")
        layout.addWidget(self.graph_sicaklik)

        # 4) Yağmur grafiği
        self.graph_toprak_nemi = pg.PlotWidget(title="Toprak Nem Grafiği")
        self._hazirla_grafik(self.graph_toprak_nemi, "Toprak Nemi (%)", "Saat")
        layout.addWidget(self.graph_toprak_nemi)

        # Tarih seçici
        self.date_picker = QDateEdit(calendarPopup=True)
        self.date_picker.setDate(QDate.currentDate())
        self.date_picker.dateChanged.connect(self.veri_listele)
        layout.addWidget(self.date_picker)

        # Günlük analiz etiketi (ışık ortalaması)
        self.gunluk_veri_label = QLabel("Günlük veri: Henüz hesaplanmadı")
        layout.addWidget(self.gunluk_veri_label)

        # Analiz tipi seçimi
        self.analiz_combo = QComboBox()
        self.analiz_combo.addItems(["Günlük", "Haftalık", "Aylık"])
        self.analiz_combo.currentTextChanged.connect(self.grafik_analiz)
        layout.addWidget(self.analiz_combo)

        # Excel'e aktar butonu
        self.excel_btn = QPushButton("Excel'e Aktar")
        self.excel_btn.setStyleSheet(
            "background-color: orange; color: black; font-weight: bold"
        )
        self.excel_btn.clicked.connect(self.excel_aktar)
        layout.addWidget(self.excel_btn)

        # Pompa kontrol butonları
        self.ac_btn = QPushButton("Pompayı Aç")
        self.kapat_btn = QPushButton("Pompayı Kapat")

        self.ac_btn.setStyleSheet(
            "background-color: green; color: white; font-weight: bold"
        )
        self.kapat_btn.setStyleSheet(
            "background-color: red; color: white; font-weight: bold"
        )

        self.ac_btn.clicked.connect(self.pompa_ac)
        self.kapat_btn.clicked.connect(self.pompa_kapat)

        pompa_layout = QHBoxLayout()
        pompa_layout.addWidget(self.ac_btn)
        pompa_layout.addWidget(self.kapat_btn)
        layout.addLayout(pompa_layout)

        self.setLayout(layout)

        # Toprak Nemi için başlangıç değeri (pompa kararı için)
        self.toprak_nem = 0.0

        # Veritabanı bağlantısı
        self.conn = sqlite3.connect("sulama_kayit.db")
        self.cursor = self.conn.cursor()
        self.cursor.execute(
            '''
            CREATE TABLE IF NOT EXISTS veriler (
                zaman    TEXT,
                nem      REAL,
                isik     REAL,
                sicaklik REAL,
                toprak_nem   REAL
            )
            '''
        )
        self.conn.commit()

        # UART / seri port
        try:
            self.seri = serial.Serial(PORT_ISMI, 9600, timeout=1)
            print("UART bağlantısı başarılı:", self.seri.port)
        except Exception as e:
            self.seri = None
            print(f"UART bağlantısı kurulamadı ({PORT_ISMI}):", e)

        # Timer – 1 saniyede bir veri oku
        self.timer = QTimer()
        self.timer.timeout.connect(self.veri_guncelle)
        self.timer.start(1000)

    # ---------------------------------------------------
    def _hazirla_grafik(self, graph, y_label, x_label):
        graph.setMinimumHeight(200)
        graph.setBackground('k')
        graph.getAxis('left').setPen('w')
        graph.getAxis('bottom').setPen('w')
        graph.getAxis('left').setTextPen('w')
        graph.getAxis('bottom').setTextPen('w')
        graph.addLegend()
        graph.showGrid(x=True, y=True)
        graph.setLabel('left', y_label)
        graph.setLabel('bottom', x_label)

        # Sensör aralıklarına göre Y eksenleri
        if ("Nem" in y_label) or ("Işık" in y_label) or ("Toprak Nem" in y_label):
            graph.setYRange(0, 100)   # % sensörler
        if "Sıcaklık" in y_label:
            graph.setYRange(0, 50)    # °C

    # Alt eksendeki özel tick'leri sıfırlamak için
    def _reset_bottom_ticks(self):
        for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
            g.getAxis('bottom').setTicks([])

    # ---------------------------------------------------
    def pompa_ac(self):
        if self.seri and self.seri.is_open:
            try:
                # yagmur yüzdelik, 30'dan küçükse pompa aç
                if self.toprak_nem < 30:
                    self.seri.write(b'1\n')
                    self.renkli_mesaj("Pompa", "Pompa açıldı.", "#28a745")
                    self.flag_open=1
                else:
                    self.renkli_mesaj("Pompa", "Topragin Nemi yüksek, pompa kapalı.", "#dc3545")
                    self.flag_open=0
            except Exception as e:
                print("Pompa AC gönderirken hata:", e)

    def pompa_kapat(self):
        if self.seri and self.seri.is_open:
            try:
                if (self.flag_open==1):
                 self.seri.write(b'0\n')
                 self.renkli_mesaj("Pompa", "Pompa kapatıldı.", "#dc3545")
                elif (self.flag_open==0):
                 self.renkli_mesaj("Pompa", "Pompa kapalı.", "#dc3545")   
            except Exception as e:
                print("Pompa KAPAT gönderirken hata:", e)

    # ---------------------------------------------------
    def veri_guncelle(self):
        zaman_dt = datetime.datetime.now()
        zaman_str = zaman_dt.strftime("%Y-%m-%d %H:%M:%S")
        self.tarih_label.setText("Tarih: " + zaman_str)

        # UART yoksa hiç uğraşma
        if not self.seri or not self.seri.is_open:
            return

        # O anda veri yoksa geç
        if self.seri.in_waiting == 0:
            return

        try:
            satir = self.seri.readline().decode(errors='ignore').strip()
            print(satir)
            if not satir:
                return

            if ',' not in satir:
                return

            nem_str, isik_str, sicaklik_str,toprak_nem_str = satir.split(',')
            nem = int(nem_str)
            isik = int(isik_str)
            sicaklik = int(sicaklik_str)
            self.toprak_nem = int(toprak_nem_str)
        except Exception as e:
            print("UART veri okuma hatası:", e)
            return

        # Işık etiketini güncelle
        self.isik_label.setText(f"Anlık Işık Değeri: {isik:.2f} %")

        # Veritabanına kaydet
        self.cursor.execute(
            "INSERT INTO veriler (zaman, nem, isik, sicaklik, toprak_nem ) VALUES (?, ?, ?, ?, ?)",
            (zaman_str, nem, isik, sicaklik, self.toprak_nem)
        )
        self.conn.commit()
      
        # Nem düşükse uyar
        if self.toprak_nem < 30 and self.flag==0:
            self.renkli_mesaj("Uyarı", "Toprak nemi çok düşük!", "#ffc107")
            self.flag=1
        elif self.flag==1 and self.toprak_nem>=30:
            self.flag=0

        # Şu anki mod ve seçili gün
        mod = self.analiz_combo.currentText()
        secili_gun = self.date_picker.date().toString("yyyy-MM-dd")

        if mod == "Günlük":
            self._gunluk_grafik_ciz(secili_gun)
        elif mod == "Haftalık":
            self._haftalik_grafik_ciz(secili_gun)
        else:  # Aylık
            self._aylik_grafik_ciz(secili_gun)

    # ---------------------------------------------------
    # GÜNLÜK: Seçilen gün için saat bazlı grafik
    # ---------------------------------------------------
    def _gunluk_grafik_ciz(self, tarih_str):
    # Seçilen günün verilerini çek
     rows = self.cursor.execute(
        "SELECT zaman, nem, isik, sicaklik, toprak_nem  FROM veriler "
        "WHERE date(zaman) = ? ORDER BY zaman;",
        (tarih_str,)
     ).fetchall()

    # --- GRAFİKLERİ TAMAMEN TEMİZLE / RESETLE ---
     for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
        g.clear()
        g.getAxis('bottom').setTicks([])      # haftalık/aylık etiketlerini sil
        g.enableAutoRange(True, True)         # autoscale tekrar açılsın

    # Eksen isimleri (hep "Saat" yazıyor, istersen mod'a göre değiştiririz)
     self._hazirla_grafik(self.graph_nem,      "Nem (%)",      "Saat")
     self._hazirla_grafik(self.graph_isik,     "Işık (%)",     "Saat")
     self._hazirla_grafik(self.graph_sicaklik, "Sıcaklık (°C)", "Saat")
     self._hazirla_grafik(self.graph_toprak_nemi,   "Toprak Nemi (%)",   "Saat")

    # Hiç veri yoksa sadece temiz kal
     if not rows:
        return

    # --------------------------------------------
    # 1) VERİLERİ GERÇEK ZAMANA GÖRE SANİYEYE ÇEVİR
    # --------------------------------------------
     gun_baslangic = datetime.datetime.strptime(tarih_str, "%Y-%m-%d")

     times_sec = []
     nem_list = []
     isik_list = []
     sicak_list = []
     yagmur_list = []

     for (zaman_txt, n, i, s, y) in rows:
        t = datetime.datetime.strptime(zaman_txt, "%Y-%m-%d %H:%M:%S")
        sec = int((t - gun_baslangic).total_seconds())  # günün kaçıncı saniyesi
        times_sec.append(sec)
        nem_list.append(n)
        isik_list.append(i)
        sicak_list.append(s)
        yagmur_list.append(y)

     min_sec = min(times_sec)
     max_sec = max(times_sec)
     span = max_sec - min_sec

    # --------------------------------------------
    # 2) MOD SEÇ: SANİYE / DAKİKA / SAAT
    # --------------------------------------------
     mode = None  # 'sec', 'min', 'hour'

    # --------- A) SADECE SANİYE (0–59 sn arası) ---------
     if span < 60:
        mode = 'sec'
        # 0'dan başlasın diye kaydırıyoruz
        x = [sec - min_sec for sec in times_sec]
        y_nem = nem_list
        y_isik = isik_list
        y_sicak = sicak_list
        y_yagmur = yagmur_list

    # --------- B) DAKİKA ORTALAMASI (1 saate kadar) -----
     elif span < 3600:
        mode = 'min'
        dakika_bin = {}  # dakika_index -> listeler

        for sec, n, i, s, y in zip(times_sec, nem_list, isik_list, sicak_list, yagmur_list):
            dk_of_day = sec // 60         # günün kaçıncı dakikası
            if dk_of_day not in dakika_bin:
                dakika_bin[dk_of_day] = {"n": [], "i": [], "s": [], "y": []}
            dakika_bin[dk_of_day]["n"].append(n)
            dakika_bin[dk_of_day]["i"].append(i)
            dakika_bin[dk_of_day]["s"].append(s)
            dakika_bin[dk_of_day]["y"].append(y)

        dk_keys = sorted(dakika_bin.keys())

        x = []
        y_nem = []
        y_isik = []
        y_sicak = []
        y_yagmur = []

        # X ekseninde 1,2,3.. şeklinde gitsin (1. dakika, 2. dakika vs)
        for idx, dk in enumerate(dk_keys, start=1):
            x.append(idx)
            y_nem.append(sum(dakika_bin[dk]["n"]) / len(dakika_bin[dk]["n"]))
            y_isik.append(sum(dakika_bin[dk]["i"]) / len(dakika_bin[dk]["i"]))
            y_sicak.append(sum(dakika_bin[dk]["s"]) / len(dakika_bin[dk]["s"]))
            y_yagmur.append(sum(dakika_bin[dk]["y"]) / len(dakika_bin[dk]["y"]))

    # --------- C) SAAT ORTALAMASI (1 saatten fazla) -----
     else:
        mode = 'hour'
        saat_bin = {}  # saat_of_day (0–23) -> listeler

        for sec, n, i, s, y in zip(times_sec, nem_list, isik_list, sicak_list, yagmur_list):
            h_of_day = (sec // 3600)  # günün kaçıncı saati (0–23)
            if h_of_day not in saat_bin:
                saat_bin[h_of_day] = {"n": [], "i": [], "s": [], "y": []}
            saat_bin[h_of_day]["n"].append(n)
            saat_bin[h_of_day]["i"].append(i)
            saat_bin[h_of_day]["s"].append(s)
            saat_bin[h_of_day]["y"].append(y)

        saat_keys = sorted(saat_bin.keys())  # sadece VERİ OLAN saatler

        x = []
        y_nem = []
        y_isik = []
        y_sicak = []
        y_yagmur = []

        for h in saat_keys:
            x.append(h)  # X ekseninde GERÇEK SAAT (0–23) olacak
            y_nem.append(sum(saat_bin[h]["n"]) / len(saat_bin[h]["n"]))
            y_isik.append(sum(saat_bin[h]["i"]) / len(saat_bin[h]["i"]))
            y_sicak.append(sum(saat_bin[h]["s"]) / len(saat_bin[h]["s"]))
            y_yagmur.append(sum(saat_bin[h]["y"]) / len(saat_bin[h]["y"]))

    # --------------------------------------------
    # 3) GRAFİĞİ ÇİZ
    # --------------------------------------------
     self.graph_nem.plot(x, y_nem,      name="Nem",      pen='c')
     self.graph_isik.plot(x, y_isik,    name="Işık",     pen='m')
     self.graph_sicaklik.plot(x, y_sicak, name="Sıcaklık", pen='y')
     self.graph_toprak_nemi.plot(x, y_yagmur,  name="Toprak Nem",   pen='b')

    # --------------------------------------------
    # 4) X EKSENİ ETİKETLERİ (TİCK) – İSTEDİĞİN KISIM
    # --------------------------------------------
     if not x:
        return

     ticks = []

     if mode == 'sec':
        # saniye: küçük span'da 1'er, biraz büyükse 5/10'ar
        max_x = max(x)
        step = 1
        if max_x > 20 and max_x <= 60:
            step = 10
        ticks = [(v, str(v)) for v in range(0, max_x + 1, step)]
        for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
            g.setLabel('bottom', 'Saniye')

     elif mode == 'min':
        # dakika: 1,2,3... N (etikette "dk" yazabiliriz)
        max_x = max(x)
        step = 1
        if max_x > 10:
            step = 5
        ticks = [(v, f"{v}") for v in range(1, max_x + 1, step)]
        for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
            g.setLabel('bottom', 'Dakika')

     elif mode == 'hour':
        # SAAT MODU: sadece VERİ OLAN saatleri etikete yaz
        # h = 0..23  ->  "0", "1", ... ya da istersen f"{h}:00"
        ticks = [(h, f"{h}") for h in sorted(set(x))]

     for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
        g.getAxis('bottom').setTicks([ticks])



    # ---------------------------------------------------
    # HAFTALIK: Seçilen tarihin bulunduğu haftanın 7 günü (Pzt–Paz)
    # ---------------------------------------------------
    def _haftalik_grafik_ciz(self, tarih_str):
        secili_tarih = datetime.datetime.strptime(tarih_str, "%Y-%m-%d").date()
        iso_yil, iso_hafta, iso_gun = secili_tarih.isocalendar()

        # ISO haftası: Pazartesi = 1
        hafta_baslangic = secili_tarih - datetime.timedelta(days=iso_gun - 1)
        gun_adlari = ["Pzt", "Sal", "Çar", "Per", "Cum", "Cmt", "Paz"]

        gun_etiketler = []
        ort_nem = []
        ort_isik = []
        ort_sicaklik = []
        ort_toprak_nem = []

        for i in range(7):
            gun_tarih = hafta_baslangic + datetime.timedelta(days=i)
            gun_str = gun_tarih.strftime("%Y-%m-%d")
            gun_adi = gun_adlari[gun_tarih.weekday()]

            sonuc = self.cursor.execute(
                "SELECT AVG(nem), AVG(isik), AVG(sicaklik), AVG(toprak_nem ) "
                "FROM veriler WHERE date(zaman) = ?;",
                (gun_str,)
            ).fetchone()

            ort_nem.append(sonuc[0] if sonuc[0] is not None else 0.0)
            ort_isik.append(sonuc[1] if sonuc[1] is not None else 0.0)
            ort_sicaklik.append(sonuc[2] if sonuc[2] is not None else 0.0)
            ort_toprak_nem.append(sonuc[3] if sonuc[3] is not None else 0.0)

            gun_etiketler.append((i, f"{gun_adi}"))

        x_index = list(range(7))

        self.graph_nem.clear()
        self.graph_isik.clear()
        self.graph_sicaklik.clear()
        self.graph_toprak_nemi.clear()

        bas_tr = hafta_baslangic.strftime("%d.%m.%Y")
        bit_tr = (hafta_baslangic + datetime.timedelta(days=6)).strftime("%d.%m.%Y")

        self._hazirla_grafik(self.graph_nem,
                             f"Ort. Nem (%)",
                             "Gün")
        self._hazirla_grafik(self.graph_isik,
                             f"Ort. Işık (%)",
                             "Gün")
        self._hazirla_grafik(self.graph_sicaklik,
                             f"Ort. Sıcaklık (°C)",
                             "Gün")
        self._hazirla_grafik(self.graph_toprak_nemi,
                             f"Ort. Toprak Nemi (%)",
                             "Gün")

        self.graph_nem.setTitle(f"Nem Grafiği (Seçili: {tarih_str} - {iso_hafta}. hafta, {bas_tr}–{bit_tr})")
        self.graph_isik.setTitle(f"Işık Grafiği (Seçili: {tarih_str} - {iso_hafta}. hafta, {bas_tr}–{bit_tr})")
        self.graph_sicaklik.setTitle(f"Sıcaklık Grafiği (Seçili: {tarih_str} - {iso_hafta}. hafta, {bas_tr}–{bit_tr})")
        self.graph_toprak_nemi.setTitle(f"Toprak Nemi Grafiği (Seçili: {tarih_str} - {iso_hafta}. hafta, {bas_tr}–{bit_tr})")

        self.graph_nem.plot(x_index, ort_nem, name="Ort. Nem", pen='c', symbol='o')
        self.graph_isik.plot(x_index, ort_isik, name="Ort. Işık", pen='m', symbol='o')
        self.graph_sicaklik.plot(x_index, ort_sicaklik, name="Ort. Sıcaklık", pen='y', symbol='o')
        self.graph_toprak_nemi.plot(x_index, ort_toprak_nem, name="Ort. Toprak Nemi", pen='b', symbol='o')

        ticks = [gun_etiketler]
        for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
            g.getAxis('bottom').setTicks(ticks)

    # ---------------------------------------------------
    # AYLIK: Ayı 4 haftaya böl, her haftanın ortalamasını göster
    # ---------------------------------------------------
    def _aylik_grafik_ciz(self, tarih_str):
        secili_tarih = datetime.datetime.strptime(tarih_str, "%Y-%m-%d").date()
        yil = secili_tarih.year
        ay = secili_tarih.month

        son_gun = calendar.monthrange(yil, ay)[1]

        # 4 hafta: 1–7, 8–14, 15–21, 22–son
        hafta_araliklari = [
            (1, 7),
            (8, 14),
            (15, 21),
            (22, son_gun)
        ]

        ort_nem = []
        ort_isik = []
        ort_sicaklik = []
        ort_toprak_nem = []
        hafta_etiketler = []

        for i, (bas_gun, bit_gun) in enumerate(hafta_araliklari, start=1):
            bas_tarih = datetime.date(yil, ay, bas_gun)
            bit_tarih = datetime.date(yil, ay, bit_gun)

            sonuc = self.cursor.execute(
                "SELECT AVG(nem), AVG(isik), AVG(sicaklik), AVG(toprak_nem ) "
                "FROM veriler WHERE date(zaman) BETWEEN ? AND ?;",
                (bas_tarih.strftime("%Y-%m-%d"),
                 bit_tarih.strftime("%Y-%m-%d"))
            ).fetchone()

            ort_nem.append(sonuc[0] if sonuc[0] is not None else 0.0)
            ort_isik.append(sonuc[1] if sonuc[1] is not None else 0.0)
            ort_sicaklik.append(sonuc[2] if sonuc[2] is not None else 0.0)
            ort_toprak_nem.append(sonuc[3] if sonuc[3] is not None else 0.0)

            hafta_etiketler.append((i - 1, f"{i}. hf"))

        x_index = list(range(4))

        self.graph_nem.clear()
        self.graph_isik.clear()
        self.graph_sicaklik.clear()
        self.graph_toprak_nemi.clear()

        self._hazirla_grafik(self.graph_nem, "Ort. Nem (%)", "Hafta")
        self._hazirla_grafik(self.graph_isik, "Ort. Işık (%)", "Hafta")
        self._hazirla_grafik(self.graph_sicaklik, "Ort. Sıcaklık (°C)", "Hafta")
        self._hazirla_grafik(self.graph_toprak_nemi, "Ort. Toprak Nemi (%)", "Hafta")

        self.graph_nem.setTitle(f"Nem Grafiği (Seçili: {tarih_str} - {ay}. ay)")
        self.graph_isik.setTitle(f"Işık Grafiği (Seçili: {tarih_str} - {ay}. ay)")
        self.graph_sicaklik.setTitle(f"Sıcaklık Grafiği (Seçili: {tarih_str} - {ay}. ay)")
        self.graph_toprak_nemi.setTitle(f"Toprak Nem Grafiği (Seçili: {tarih_str} - {ay}. ay)")

        self.graph_nem.plot(x_index, ort_nem, name="Ort. Nem", pen='c', symbol='o')
        self.graph_isik.plot(x_index, ort_isik, name="Ort. Işık", pen='m', symbol='o')
        self.graph_sicaklik.plot(x_index, ort_sicaklik, name="Ort. Sıcaklık", pen='y', symbol='o')
        self.graph_toprak_nemi.plot(x_index, ort_toprak_nem, name="Ort. Toprak Nemi", pen='b', symbol='o')

        ticks = [hafta_etiketler]
        for g in [self.graph_nem, self.graph_isik, self.graph_sicaklik, self.graph_toprak_nemi]:
            g.getAxis('bottom').setTicks(ticks)

    # ---------------------------------------------------
    def veri_listele(self):
        secilen_tarih = self.date_picker.date().toString("yyyy-MM-dd")

        result = self.cursor.execute(
            "SELECT AVG(isik) FROM veriler WHERE date(zaman) = ?;",
            (secilen_tarih,)
        ).fetchone()
        ort_isik = result[0] if result[0] is not None else 0.0
        self.gunluk_veri_label.setText(
            f"{secilen_tarih} tarihinde ortalama ışık: {ort_isik:.2f} %"
        )

        mod = self.analiz_combo.currentText()
        if mod == "Günlük":
            self._gunluk_grafik_ciz(secilen_tarih)
        elif mod == "Haftalık":
            self._haftalik_grafik_ciz(secilen_tarih)
        else:
            self._aylik_grafik_ciz(secilen_tarih)

    # ---------------------------------------------------
    def grafik_analiz(self, secim):
        secilen_tarih = self.date_picker.date().toString("yyyy-MM-dd")
        if secim == "Günlük":
            self._gunluk_grafik_ciz(secilen_tarih)
        elif secim == "Haftalık":
            self._haftalik_grafik_ciz(secilen_tarih)
        else:
            self._aylik_grafik_ciz(secilen_tarih)

    # ---------------------------------------------------
    def excel_aktar(self):
        try:
            df = pd.read_sql_query("SELECT * FROM veriler", self.conn)
            dosya_yolu, _ = QFileDialog.getSaveFileName(
                self, "Excel'e Aktar", "veriler.xlsx", "Excel Dosyası (*.xlsx)"
            )
            if dosya_yolu:
                df.to_excel(dosya_yolu, index=False)
        except Exception as e:
            print("Excel aktarım hatası:", e)

    # ---------------------------------------------------
    def renkli_mesaj(self, baslik, mesaj, renk):
        msg = QMessageBox(self)
        msg.setWindowTitle(baslik)
        msg.setText(mesaj)
        msg.setStyleSheet(
            f"QPushButton {{ background-color: {renk}; color: white; "
            f"font-weight: bold; padding: 6px 12px; }}"
        )
        msg.exec_()

    # ---------------------------------------------------
    def closeEvent(self, event):
        if self.seri:
            try:
                self.seri.close()
            except:
                pass
        self.conn.close()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    pencere = SulamaArayuz()
    pencere.show()
    sys.exit(app.exec_())
