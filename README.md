# SIM7028 Bridge v26.29 – Konstante Spannungsversorgung (KEIN Solar!)

SIM800 → SIM7028 Translator Bridge (1NCE, APN `iot.1nce.net`)
Hardware: M5Stack ATOM DTU NB-IoT2 + AtomS3 Lite

**Nur fuer Betrieb an konstanter Spannungsversorgung (Netzteil/Dauerstrom).**
Nicht fuer Solar-/Akkubetrieb geeignet: Modem-Stromsparmodi sind abgeschaltet
(AT+CSCLK=0, AT+CPSMS=0, AT+CEDRXS=0), der ESP32 schlaeft nicht (Ausnahme:
1 Min Deep Sleep beim taeglichen Neustart).

Quelle: `Downloads/SIM7028_Bridge_v26.29/SIM7028_Bridge_v26.29.ino`
