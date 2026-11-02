"""Motor + kayış sürtünmesinin ölçümü (cihazdaki "friction" komutu; firmware: frictionStep).

Araba dururken motor gerilimi yavaşça artırılır. Hız, FR_WINDOW süreli pencerelerde yer değiştirmeden
hesaplanır (anlık hız kestirimi enkoder adımları yüzünden gürültülüdür) ve her pencerenin ortalama
gerilimiyle eşlenir. Hızı FR_V_LO…FR_V_HI aralığındaki pencerelere doğru oturtulur:

    V = V_c + k·v        V_c: kayma (Coulomb) sürtünmesini yenen gerilim [V],  k: (β + b)/α [V·s/m]

Önce +x, araba durunca −x yönünde ölçülür. Eğim k iki yönde ortaktır (motor sabitleri yönden
bağımsız); her yönün kendi V_c'si bulunur ve ikisinin ortalaması alınır. Her yönün sonunda gerilim
yavaşça indirilir (ani duruş sarkacı sallar, sallanan sarkaç da arabayı iter ve sonraki ölçümü bozar).

Sonuç GERİLİM olarak verilir: telafi de gerilime çevrilerek uygulandığı için (accelToVolt, α'ya bölünür)
motor sabitlerindeki hata sonucu etkilemez. Eğim k, modeldeki (β + b_cart)/α ile karşılaştırılıp motor
sabitleri kontrol edilebilir. Rampa yavaş olduğu için ivme payı küçüktür (1 V/s'de V_c birkaç % fazla
çıkabilir). Gerçek düzenekte sürtünme konuma ve yöne göre değişebilir; sonuç ilk denemede kontrol edilmelidir.
"""

from __future__ import annotations

import math

from ..model.dynamics import wrap_angle

FR_RATE = 1.0            # gerilim rampası [V/s]
FR_STOP_RATE = 2.0       # her yönün sonunda gerilimin indirilme hızı [V/s]
FR_WINDOW = 0.04         # hızın yer değiştirmeden hesaplandığı pencere [s]
FR_V_LO = 0.04           # doğru oturtmada kullanılan en düşük araba hızı [m/s]
FR_V_HI = 0.10           # bu hıza ulaşınca o yöndeki ölçüm biter [m/s]
FR_MAX_VOLT = 6.0        # rampanın en yüksek gerilimi [V]
FR_TRAVEL = 0.08         # yön başına ölçüm yolu sınırı [m] (yumuşak duruş ~3 cm daha gider)
FR_X_MARGIN = 0.15       # başlarken araba ray ucundan en az FR_TRAVEL + bu kadar uzak olmalı [m]
FR_TIMEOUT = 8.0         # yön başına süre sınırı [s]
FR_SETTLE = 0.3          # yön değiştirmeden önce araba bu kadar süre FR_STILL_DIST'ten az hareket etmeli [s]
FR_STILL_DIST = 0.002    # [m]
FR_SETTLE_TIMEOUT = 3.0  # [s]
FR_MIN_SAMPLES = 5       # doğru oturtmak için en az pencere
FR_MIN_SPAN = 0.03       # pencerelerin kapsaması gereken hız aralığı [m/s]
FR_SWING_MAX = 0.3       # sarkaç aşağıdan bu kadar [rad] saparsa ölçüm iptal
FR_STILL_DTH = 0.3       # başlarken sarkacın açısal hızı bundan küçük olmalı [rad/s]
FR_STILL_DX = 0.01       # başlarken araba hızı bundan küçük olmalı [m/s]


def start_block_reason(p, x: float, dx: float, th: float, dth: float) -> str:
    """Ölçüm başlayabilir mi? Uygunsa boş metin, değilse sebebi (firmware ile aynı metinler)."""
    if p.input_mode != "voltage":
        return "sürtünme ölçümü yalnız DC motorda (voltage)"
    if abs(x) > p.rail_half - FR_TRAVEL - FR_X_MARGIN:
        return "araba ortada değil: önce KALİBRE ET"
    if abs(dx) > FR_STILL_DX:
        return "araba hareket ediyor: durmasını bekleyin"
    if abs(math.pi - abs(wrap_angle(th))) > FR_SWING_MAX or abs(dth) > FR_STILL_DTH:
        return "sarkaç aşağıda ve hareketsiz olmalı"
    return ""


class FrictionTest:
    """Ölçümün adım adım yürütülmesi. step() her kontrol periyodunda çağrılır; (u [V], sonuç) döndürür.
    Sonuç ölçüm sürerken None, bitince {"ok": True, "v_pos", "v_neg", "v_c", "slope"} veya
    {"ok": False, "reason"}."""

    def __init__(self, x0: float):
        self.direction = 1
        self.sums: list[tuple] = []             # her yön için (n, Σv, ΣV, Σv², ΣvV)
        self._begin_ramp(x0)

    def _begin_ramp(self, x: float):
        self.phase, self.t, self.volt, self.x0 = "ramp", 0.0, 0.0, x
        self.n = 0
        self.sv = self.sV = self.svv = self.svV = 0.0
        self.v_min, self.v_max = math.inf, -math.inf
        self._window(x)

    def _window(self, x: float):
        self.w_t, self.w_V, self.w_x0 = 0.0, 0.0, x

    def _enough(self) -> bool:
        return self.n >= FR_MIN_SAMPLES and self.v_max - self.v_min >= FR_MIN_SPAN

    def _result(self) -> dict:
        """Ortak eğimli en küçük kareler: V = V_c(yön) + k·v."""
        Svv = SvV = 0.0
        for n, sv, sV, svv, svV in self.sums:
            Svv += svv - sv * sv / n
            SvV += svV - sv * sV / n
        if Svv <= 0.0:
            return {"ok": False, "reason": "yeterli hıza ulaşılamadı (gerilim/yol/süre sınırı)"}
        k = SvV / Svv
        vp, vn = ((sV - k * sv) / n for n, sv, sV, _, _ in self.sums)
        return {"ok": True, "v_pos": vp, "v_neg": vn, "v_c": (vp + vn) / 2, "slope": k}

    def step(self, x: float, dx: float, th: float, dth: float, dt: float):
        if abs(math.pi - abs(wrap_angle(th))) > FR_SWING_MAX:
            return 0.0, {"ok": False, "reason": "sarkaç sallandı"}
        self.t += dt
        if self.phase == "stop":                 # gerilimi yavaşça indir
            self.volt = max(0.0, self.volt - FR_STOP_RATE * dt)
            if self.volt > 0.0:
                return self.direction * self.volt, None
            if len(self.sums) == 2:
                return 0.0, self._result()
            self.phase, self.t = "settle", 0.0
            self._window(x)
            return 0.0, None
        if self.phase == "settle":               # FR_SETTLE boyunca FR_STILL_DIST'ten az hareket
            self.w_t += dt
            if abs(x - self.w_x0) > FR_STILL_DIST:
                self._window(x)
            elif self.w_t >= FR_SETTLE:
                self.direction = -1
                self._begin_ramp(x)
            elif self.t > FR_SETTLE_TIMEOUT:
                return 0.0, {"ok": False, "reason": "araba durmadı"}
            return 0.0, None

        # rampa: pencere dolunca (ortalama gerilim, yer değiştirmeden hız) örneği
        speed = None
        if self.w_t >= FR_WINDOW - 1e-9:
            speed = self.direction * (x - self.w_x0) / self.w_t
            volt = self.w_V / self.w_t
            if FR_V_LO <= speed <= FR_V_HI:
                self.n += 1
                self.sv += speed
                self.sV += volt
                self.svv += speed * speed
                self.svV += speed * volt
                self.v_min, self.v_max = min(self.v_min, speed), max(self.v_max, speed)
            self._window(x)
        if ((speed is not None and speed >= FR_V_HI) or abs(x - self.x0) > FR_TRAVEL
                or self.volt >= FR_MAX_VOLT or self.t > FR_TIMEOUT):
            if not self._enough():
                return 0.0, {"ok": False, "reason": "yeterli hıza ulaşılamadı (gerilim/yol/süre sınırı)"}
            self.sums.append((self.n, self.sv, self.sV, self.svv, self.svV))
            self.phase = "stop"
            return self.direction * self.volt, None
        self.volt = min(self.volt + FR_RATE * dt, FR_MAX_VOLT)
        self.w_t += dt                           # bu adımda uygulanan gerilim penceredeki ortalamaya girer
        self.w_V += self.volt * dt
        return self.direction * self.volt, None
