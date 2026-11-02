# Fiziksel Parametre Tablosu (mekatronik ekibi dolduracak)

Mekanik tasarım bitince bu tablo doldurulup bilgisayar ekibine iletilir. Değerler `config/params.json` dosyasına (aynı alan adlarıyla) yazılır. Şimdilik kullanılan değerler **yer tutucudur**. Ölçülmüş değerler girilince dosyadaki `"placeholder": true` satırı `false` yapılmalıdır (ya da arayüzün Tasarım sekmesinde **Ölçülmüş gerçek değerler** kutusu işaretlenip **Parametre Kaydet**). O zamana kadar arayüz gerçek karta K'yı onay almadan göndermez ve `gains.h` derlenirken uyarı verir.

| Alan | Açıklama | Birim | Yer tutucu | Gerçek değer | Nasıl ölçülür / bulunur |
|---|---|---|---|---|---|
| `M` | Arabanın hareket eden toplam kütlesi (araba + kayış payı + sarkaç mili) | kg | 0.5 | | Terazi |
| `m` | Sarkacın toplam kütlesi (çubuk + uç kütle) | kg | 0.2 | | Terazi |
| `l` | Pivot ekseninden sarkacın **ağırlık merkezine** uzaklık | m | 0.3 | | Bıçak sırtında dengeleme veya CAD (Inventor) |
| `I` | Sarkacın **kendi ağırlık merkezi** etrafındaki ataleti | kg·m² | 0 | | Inventor "iProperties → Physical" veya arayüzdeki geometri hesaplayıcı |
| `b` | Araba viskoz sürtünmesi | N·s/m | 0.1 | | Arabayı itip serbest bırakın, hız azalma eğrisinden |
| `c` | Pivot (rulman) sürtünmesi | N·m·s/rad | 0.0005 | | Sarkacı aşağıda salınıma bırakın, genlik sönümünden |
| `rail_half` | Ray ortasından limit switch'e uzaklık | m | 0.40 | | Metre |
| `u_max` | Motor gerilim sınırı | V | 12 | | Güç kaynağı |
| `input_mode` | `voltage` (DC motor), `acceleration` (step motor), `force` | — | voltage | | Motor seçimi |
| `motor.R` | Motor sargı direnci | Ω | 2.2 | | Datasheet veya ohmmetre |
| `motor.Kt` | Tork sabiti (**motor mili**, redüktör öncesi) | N·m/A | 0.0083 | | Datasheet: kilitli tork / kilitli akım / redüktör oranı |
| `motor.Ke` | Zıt EMK sabiti (motor mili) | V·s/rad | 0.0109 | | Datasheet: (V − I₀R) / boşta hız (rad/s, motor mili) |
| `motor.gear` | Redüktör oranı | — | 10 | | Ürün adı (ör. 10:1) |
| `motor.r_pulley` | Kayış kasnağı yarıçapı | m | 0.0127 | | GT2: diş sayısı × 2 mm / 2π |
| `motor.V_max` | Besleme gerilimi | V | 12 | | `u_max` bundan büyük olamaz |
| `motor.J_rotor` | Rotor ataleti (motor mili) | kg·m² | 0 | | Datasheet. Arabaya J·N²/r² kadar eşdeğer kütle ekler — redüktörlü motorda araba kütlesiyle kıyaslanabilir olabilir |
| `motor.T_f` | Coulomb (kuru) sürtünme torku (motor mili) | N·m | 0 | | Boşta akımdan: T_f ≈ Kt·I₀ (Pololu 37D: 0.0089 × 0.2 A ≈ 0.0018). Arabaya T_f·N/r kadar sabit sürtünme kuvveti olarak yansır |
| `motor.B_m` | Viskoz sürtünme (motor mili) | N·m·s/rad | 0 | | Boşta akım ve hızdan: B_m ≈ Kt·I₀ / ω₀ (Pololu 37D: ≈ 1.7·10⁻⁶). Arabaya B_m·N²/r² kadar sönüm ekler |

**T_f ve B_m için not:** İkisi de aynı boşta akım kaybından (Kt·I₀) hesaplanırsa aynı kayıp iki kez sayılır. Ya biri seçilmeli ya da kayıp ikisine bölünmelidir (ör. iki farklı gerilimde boşta akım ölçülerek: hızla değişmeyen kısım T_f, hızla artan kısım B_m). Kayış-ray sürtünmesi motorun değil arabanın sürtünmesidir, `b`'ye yazılır.

## Motor seçimi için not
Tasarım sekmesi, seçilen motorla ulaşılabilecek **en yüksek araba hızını** gösterir. Örnek: Pololu 37D, 30:1 redüktör ve 20 dişli GT2 kasnakla araba en fazla ~0.2 m/s gidebilir. Bu, swing-up için **yetersizdir**. Yer tutucu olarak 10:1 redüktör ve 40 dişli kasnak (~1.4 m/s) kullanıldı.
Step motor (NEMA17) seçilirse `input_mode = "acceleration"` yapılır. Bu durumda firmware'de motor sürme kısmı ivme → adım frekansı olarak yeniden yazılmalıdır.

## Küre mi küp mü?
Arayüzdeki "Geometri hesaplayıcı" ile iki seçenek karşılaştırılabilir. Aynı kütlede, küp kenarı küre çapına eşitse küpün ataleti 1/6·m·a²'dir; küreninki 2/5·m·r² = 1/10·m·d²'dir. Uç kütle küçük ve çubuk uzunsa fark ihmal edilebilir: J = I + m·l² içinde m·l² baskındır. Kontrol açısından belirleyici olan **l** (ağırlık merkezinin pivota uzaklığı) ve toplam kütledir. Şekil seçimi daha çok montaj kolaylığına göre yapılabilir.
