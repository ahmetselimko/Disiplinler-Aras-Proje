# Fiziksel Parametre Tablosu (mekatronik ekibi ölçüp dolduracak)

Mekanik tasarım bitince bu tablo doldurulup bilgisayar ekibine iletilir. Değerler `config/params.json` dosyasına (aynı alan adlarıyla) yazılır. **#50 Aşama 2'den beri dosyada mekatronik ara raporunun (23.11.2026) TASARIM değerleri var; ölçüm değil.** Bu yüzden `"placeholder": true` kalıyor. Ölçülmüş değerler girilince `false` yapılmalıdır (ya da arayüzün Tasarım sekmesinde **Ölçülmüş gerçek değerler** kutusu işaretlenip **Parametre Kaydet**). O zamana kadar arayüz gerçek karta K'yı onay almadan göndermez ve `gains.h` derlenirken uyarı verir.

Uç kütlesi kararı bekleniyor; üç seçenek ayrı dosyalarda, arayüzde **Yükle** ile seçilir: `params.json` (**+20 g, varsayılan**), `params_ucsuz.json` (uç kütlesi yok), `params_50g.json` (+50 g). Tabloda varsayılan dosyanın değeri, parantez içinde uçsuz / +50 g.

| Alan | Açıklama | Birim | Şimdiki değer (tasarım) | Kaynak | Gerçek değer | Nasıl ölçülür / bulunur |
|---|---|---|---|---|---|---|
| `M` | Arabanın hareket eden toplam kütlesi (araba + kayış + kasnaklar + sarkaç mili; sarkaç **hariç**) | kg | 1.28 (1.30 / 1.25) | Rapor Bölüm 12 ve 15: araba 1.10 kg sarkaç **dahil** → 1.10 − m + 0.26 (kayış, 14.1) + 0.025 (kasnaklar, 14.1) | | Terazi |
| `m` | Sarkacın toplam kütlesi (çubuk + uç kütle) | kg | 0.105 (0.085 / 0.135) | Tablo 15 | | Terazi |
| `l` | Pivot ekseninden sarkacın **ağırlık merkezine** uzaklık | m | 0.0992 (0.0637 / 0.1327) | Tablo 15 | | Bıçak sırtında dengeleme veya CAD (Inventor) |
| `I` | Sarkacın **kendi ağırlık merkezi** etrafındaki ataleti | kg·m² | 0.001203 (0.000641 / 0.001734) | Tablo 15'teki I_pivot'tan: I = I_pivot − m·l² | | Inventor "iProperties → Physical" veya arayüzdeki geometri hesaplayıcı |
| `b` | Araba viskoz sürtünmesi | N·s/m | 0.1 | Yer tutucu (rapor 3 N'luk sürtünmeyi Coulomb olarak veriyor: `motor.T_f`) | | Arabayı itip serbest bırakın, hız azalma eğrisinden |
| `c` | Pivot (rulman) sürtünmesi | N·m·s/rad | 0.0005 | Yer tutucu | | Sarkacı aşağıda salınıma bırakın, genlik sönümünden |
| `rail_half` | Ray ortasından limit switch'e uzaklık | m | 0.40 | Geçici, iki ekipte aynı (rapor 8.4); şasi 2.18 m, gerçek yol sorulacak | | Metre |
| `u_max` | Motor gerilim sınırı | V | 12 | Rapor 8.1.5 (12 V besleme) | | Güç kaynağı |
| `input_mode` | `voltage` (DC motor), `acceleration` (step motor), `force` | — | voltage | DC motor | | Motor seçimi |
| `motor.R` | Motor sargı direnci | Ω | 9.478 | Tablo 16'dan türetildi: R = b/α² = 48.41 / 2.26² (Kt = Ke varsayımıyla; `params_mekatronik_tasarim.json` ile aynı). Yalnız α ve b önemli | | Datasheet veya ohmmetre |
| `motor.Kt` | Tork sabiti (**motor mili**, redüktör öncesi) | N·m/A | 0.0107654 | Tablo 16: α = Kt·N/(R·r) = 2.26 N/V | | Datasheet: kilitli tork / kilitli akım / redüktör oranı |
| `motor.Ke` | Zıt EMK sabiti (motor mili) | V·s/rad | 0.0107654 | Tablo 16: b_elec = Kt·Ke·N²/(R·r²) = 48.41 N·s/m | | Datasheet: (V − I₀R) / boşta hız (rad/s, motor mili) |
| `motor.gear` | Redüktör oranı | — | 19 | Bölüm 13.3 (JGB37-520 19:1) | | Ürün adı |
| `motor.r_pulley` | Kayış kasnağı yarıçapı | m | 0.009549 | Tablo 16 (HTD3M 20 diş: 20·3 mm / 2π) | ✓ | HTD3M: diş sayısı × 3 mm / 2π. GT2'de × 2 mm |
| `motor.V_max` | Besleme gerilimi | V | 12 | Rapor 8.1.5 | | `u_max` bundan büyük olamaz |
| `motor.J_rotor` | Rotor ataleti (motor mili) | kg·m² | 3.4e-7 | Bölüm 14.1: arabaya yansıyan 1.346 kg → J = 1.346·r²/N² | | Datasheet. Arabaya J·N²/r² kadar eşdeğer kütle ekler |
| `motor.T_f` | Coulomb (kuru) sürtünme torku (motor mili) | N·m | 0.0015077 | Bölüm 14.1: 3.0 N araba sürtünmesi → T_f = 3·r/N | | Arayüzdeki **Sürtünme Ölç** ya da boşta akımdan T_f ≈ Kt·I₀. Arabaya T_f·N/r kadar sabit sürtünme kuvveti olarak yansır |
| `motor.B_m` | Viskoz sürtünme (motor mili) | N·m·s/rad | 0 | — | | Boşta akım ve hızdan: B_m ≈ Kt·I₀ / ω₀. Arabaya B_m·N²/r² kadar sönüm ekler |

Kontrol: Tasarım sekmesi bu değerlerle α = 2.260 N/V, β = 48.41 N·s/m, en yüksek araba hızı ≈ 0.50 m/s (sürtünmesiz 0.56 m/s; 3 N sürtünme düşülmüş) ve durma kuvveti ≈ 27.1 N gösterir (rapor Tablo 16: 0.53 m/s yüksüz, 27.07 N).

**Enkoder:** JGB37-520'de 836 sayım / çıkış mili turu (Bölüm 13.3) → motor milinde 836 / 19 = **44 CPR** (4x sayım). Firmware'de `ENC_CPR_MOTOR`, Python'da `filters.ENC_CPR_MOTOR`. Araba konumunun bir sayımı 2π·r / (44 · 19) = 60 mm / 836 ≈ **0.072 mm**. Motor gelince doğrulanmalı.

**T_f ve B_m için not:** İkisi de aynı boşta akım kaybından (Kt·I₀) hesaplanırsa aynı kayıp iki kez sayılır. Ya biri seçilmeli ya da kayıp ikisine bölünmelidir (ör. iki farklı gerilimde boşta akım ölçülerek: hızla değişmeyen kısım T_f, hızla artan kısım B_m). Kayış-ray sürtünmesi motorun değil arabanın sürtünmesidir, `b`'ye yazılır.

## Motor seçimi için not
**Seçilen motor: JGB37-520 12 V 530 rpm, 19:1** (mekatronik ara raporu, son seçim; BOM v3'teki diğer aday DFRobot FIT0186 idi). Motor değerleri raporun Tablo 16'sındaki gerilim–kuvvet kazancı ve zıt EMK sönümünden türetildi (yukarıdaki tablo); datasheet gelince `tools/motor_karsilastirma.py`'deki formüllerle kontrol edilmeli.

Tasarım sekmesi, seçilen motorla ulaşılabilecek **en yüksek araba hızını** gösterir. Örnek (eski aday): Pololu 37D, 30:1 redüktör ve 20 dişli GT2 kasnakla araba en fazla ~0.2 m/s gidebilir; bu, swing-up için **yetersizdir**. JGB37-520 + 20 diş ≈ 0.50 m/s (3 N sürtünmeyle) verir ve simülasyonda +20 g ve +50 g uç kütlesiyle swing-up için yetiyor (`mekatronik_on_degerlendirme.md`).
Step motor (NEMA17) seçilirse `input_mode = "acceleration"` yapılır. Bu durumda firmware'de motor sürme kısmı ivme → adım frekansı olarak yeniden yazılmalıdır.

## Küre mi küp mü?
Arayüzdeki "Geometri hesaplayıcı" ile iki seçenek karşılaştırılabilir. Aynı kütlede, küp kenarı küre çapına eşitse küpün ataleti 1/6·m·a²'dir; küreninki 2/5·m·r² = 1/10·m·d²'dir. Uç kütle küçük ve çubuk uzunsa fark ihmal edilebilir: J = I + m·l² içinde m·l² baskındır. Kontrol açısından belirleyici olan **l** (ağırlık merkezinin pivota uzaklığı) ve toplam kütledir. Şekil seçimi daha çok montaj kolaylığına göre yapılabilir.
