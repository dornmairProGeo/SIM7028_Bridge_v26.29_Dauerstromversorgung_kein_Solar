/**
 * =============================================================================
 * SIM800 → SIM7028 Translator Bridge  v26.29  (1NCE / iot.1nce.net)
 *
 * Hardware: M5Stack ATOM DTU NB-IoT2 + AtomS3 Lite
 *
 * UART-Belegung:
 *   USB-CDC  (Serial)   = Debug   115200
 *   UART1    (Serial1)  = SIM7028  RX:G6  TX:G5   115200
 *   UART2    (Serial2)  = Ext. µC  RX:G39 TX:G38    9600
 *
 * Version History:
 *   v26.29 (2026-11-09) – Nichtfluechtiges Fehler-Log im Flash
 *     FIX 48 – Fehler und wichtige Ereignisse werden in LittleFS
 *       (/errlog.txt, max. 64 KB, danach Rotation nach /errlog_old.txt)
 *       gespeichert und ueberleben Neustart und Stromausfall.
 *       Jeder Eintrag: Boot-Nr. (NVS-Zaehler), Netzzeit (AT+CCLK, falls
 *       vom Netz geliefert), Uptime, Level (BOOT/INFO/WARN/ERR), Text.
 *       Erfasst u.a.: Reset-Grund (Brownout/Watchdog/Panic), AT-ERROR/
 *       TIMEOUT, Modem tot/haengt, Registrierung (ok/abgelehnt/Backoff/
 *       aufgegeben), SAPBR-Fehler, HTTP-Fehler, Haenger- und Tages-
 *       Neustart inkl. Tagesbilanz (HTTP ok/Fehler).
 *       Schutz vor Flash-Verschleiss: max. 40 Eintraege pro 10 Min.
 *       Auslesen per USB: LOG (anzeigen), LOGCLR (loeschen).
 *       Ext-UART RX-Puffer auf 1024B vergroessert (Flash-Schreibzeit).
 *
 *   v26.28 (2026-11-09) – Taeglicher Neustart von ATOM + SIM7028
 *     FIX 47 – Alle 24h Betriebszeit:
 *       1. Warten, bis der ext. µC mind. 2 Min still ist und kein HTTP
 *          laeuft (max. 2h verschieben, dann erzwungen).
 *       2. SIM7028 sauber abschalten (AT+CPOWD=1 ans Modem; Fallback
 *          AT+CFUN=0, falls kein "NORMAL POWER DOWN" kommt).
 *       3. ATOM geht fuer 60s in Deep Sleep (= aus, 1 Min Pause).
 *       4. Timer-Wakeup = kompletter Kaltstart: setup() schaltet das Modem
 *          per Power-Key wieder ein (Fallback: zweiter, langer 1s-Puls).
 *       Power-Key-Pin wird waehrend des Sleeps per GPIO-Hold auf LOW
 *       gehalten (kein ungewollter Puls). USB-Befehl "DAILY" loest den
 *       Ablauf sofort zum Testen aus. Zaehler ueberlebt den Deep Sleep.
 *
 *   v26.27 (2026-11-02) – Kein Schlafen nach dem Senden + 5-Min-Haenger-Neustart
 *     FIX 45 – Bridge/Modem bleiben nach erfolgreichem Senden wach:
 *       - Beim Boot werden alle Modem-Stromsparmodi explizit abgeschaltet:
 *         AT+CSCLK=0 (kein UART-Sleep), AT+CPSMS=0 (kein PSM),
 *         AT+CEDRXS=0 (kein eDRX). Das Modem geht damit nach dem Senden
 *         nicht mehr in einen Schlafzustand, aus dem es per UART nicht
 *         sofort erreichbar ist.
 *       - AT+CPOWD=1 vom µC setzt cfunFull/netRegistered/regState NICHT mehr
 *         zurueck: das Modem wird (seit FIX 33) ohnehin nicht abgeschaltet,
 *         bleibt eingebucht und ist beim naechsten Zyklus sofort bereit.
 *       - Der ESP32 nutzt weiterhin keinerlei Light-/Deep-Sleep.
 *       - +HTTP_NONET_EVENT setzt regState jetzt auf REG_NOT_STARTED, damit
 *         beim naechsten AT+SAPBR=1,1 neu registriert wird (vorher blieb
 *         regState faelschlich auf REGISTERED).
 *     FIX 46 – Neustart nach 5 Minuten Haenger (zweistufig):
 *       a) ESP32 Task-Watchdog (TWDT) mit 300s Timeout auf dem loop()-Task.
 *          Alle Warteschleifen fuettern ihn ueber wdtYield(); haengt der
 *          Code ausserhalb dieser (begrenzten) Schleifen fest, loest der
 *          TWDT nach 5 Min einen Hardware-Reset aus.
 *       b) Modem-Lebenszeichen-Ueberwachung: jede gueltige Modem-Antwort
 *          (und jeder URC) setzt lastModemAliveMs. Im Leerlauf wird alle
 *          60s ein "AT" geschickt. Antwortet das Modem 5 Min lang gar
 *          nicht, startet die Bridge per ESP.restart() neu (setup() versucht
 *          dann AT und bei Bedarf den Power-Key-Puls).
 *       Der Reset-Grund wird beim Boot wie gehabt angezeigt
 *       (Task-Watchdog-Timeout bzw. Software-Reset).
 *
 *   v26.26 (2026-07-16) – FIX 44: AT+HTTPACTION=0 antwortet dem µC ERST NACH
 *       fertigem sim7028HttpGet() mit OK (synchron) – behebt Ein-Zyklus-
 *       Versatz des HTTPREAD-Payloads.
 *   v26.25 (2026-07-15) – FIX 43: "+HTTPREAD: 0"-Trailer entfernt, Block
 *       endet byte-genau wie SIM800 ("...<payload>\r\nOK").
 *   v26.24 (2026-07-15) – FIX 42: +HTTPACTION + +HTTPREAD + Payload + OK als
 *       EIN Block (Sniffer-verifiziertes SIM800-Format).
 *   v26.23 (2026-07-15) – FIX 41: kompletter xx-Payload-Parser im Log.
 *
 *   ----------------------------------------------------------------------------
 *   Aeltere Eintraege (v26.22 und frueher) gekuerzt - Kurzreferenz:
 *     FIX 38-40  HTTPREAD-Antwort als ein Block, "OK"/'K'-Trigger fuer µC
 *     FIX 29/37  kompletter Modem-Rohtext (rawHttpReadBuf), Beginn ab
 *                "+HTTPREAD:"-Marker (fruehes Ack-OK abschneiden)
 *     FIX 33/35  AT+CPOWD=1 nicht ans Modem, Watchdog-Shutdown (Brownout)
 *     FIX 26.x   1NCE No-Harm Backoff, Fast-Toggle, SAPBR-Zeitbudget
 *     FIX 23/24  Hintergrund-Registrierung (µC 13s-Timeout), Exp.-Backoff
 *     FIX 22     FPLMN-Reset beim Boot
 *     FIX 19/20/21 HTTPREAD live vom Modem, Boot-Cleanup, CPOWD ERROR ok
 *     FIX 16/17/18 kein Brownout-Disable, PDP-Kontext 0, Operator-Fallback
 *   ----------------------------------------------------------------------------
 * =============================================================================
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "esp_system.h"   // FIX 26: esp_reset_reason() fuer Boot-Diagnose
#include "esp_task_wdt.h" // FIX 46: Task-Watchdog (5-Min-Haenger-Neustart)
#include "esp_sleep.h"     // FIX 47: Deep Sleep als 1-Min-Pause
#include "driver/gpio.h"  // FIX 47: GPIO-Hold fuer Power-Key im Sleep
#include <LittleFS.h>     // FIX 48: Fehler-Log im Flash
#include <Preferences.h>  // FIX 48: Boot-Zaehler im NVS
#include <time.h>

#define FW_VERSION  "v26.29"
#define FW_DATE     "2026-11-09"

// ── Pins ──────────────────────────────────────────────────────────────────────
#define SIM_TX_PIN    5
#define SIM_RX_PIN    6
#define SIM_PWR_PIN   7
#define EXT_TX_PIN   38
#define EXT_RX_PIN   39
#define LED_PIN      35
#define NUM_LEDS      1

Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── Baudrates ─────────────────────────────────────────────────────────────────
#define SIM_BAUD    115200
#define EXT_BAUD      9600

// ── APN ───────────────────────────────────────────────────────────────────────
#define APN  "iot.1nce.net"
//#define APN "m2m.melita.io"

// ── PDP-Kontext (FIX 17: SIM7028 nutzt Kontext 0 für die IP) ──────────────────
#define PDP_CID  0

// ── FIX 46: Haenger-Ueberwachung ──────────────────────────────────────────────
#define HANG_RESTART_S           300U        // 5 Minuten
#define HANG_RESTART_MS          (HANG_RESTART_S * 1000UL)
#define MODEM_HEALTH_INTERVAL_MS 60000UL     // Leerlauf-Pruefung alle 60s

// ── FIX 47: Taeglicher Neustart ───────────────────────────────────────────────
#define DAILY_RESTART_MS        (24UL * 3600UL * 1000UL) // alle 24h Betriebszeit
#define DAILY_RESTART_OFF_S     60ULL                    // 1 Min Pause (aus)
#define DAILY_RESTART_IDLE_MS   120000UL                 // µC mind. 2 Min still
#define DAILY_RESTART_MAX_DELAY_MS (2UL * 3600UL * 1000UL) // max. 2h verschieben

// ── DE-Netzbetreiber für Fallback (FIX 18) ────────────────────────────────────
struct Operator { const char *plmn; const char *name; };
static const Operator DE_OPERATORS[] = {
    { "26201", "Telekom DE" },
    { "26202", "Vodafone DE" },
    { "26203", "o2 / Telefonica DE" },
};
static const int NUM_OPERATORS = sizeof(DE_OPERATORS)/sizeof(DE_OPERATORS[0]);

// ── Timeouts ──────────────────────────────────────────────────────────────────
#define T_AT       800U
#define T_SHORT   3000U
#define T_CFUN   18000U
#define T_MEDIUM  8000U
#define T_QUICK   2000U
#define T_NETREG 180000U   // 3 Minuten Auto-Registrierung (nur manueller "REG")
#define T_OPREG   90000U   // 90s pro manuellem Operator-Versuch (dito)
#define T_HTTP   120000U

#define EXT_UC_HARD_TIMEOUT_MS    13000U
#define EXT_UC_SAFETY_MARGIN_MS     800U
#define SAPBR_WAIT_MS  (EXT_UC_HARD_TIMEOUT_MS - EXT_UC_SAFETY_MARGIN_MS) // = 12200ms
#define REG_BG_POLL_MS         3000U
#define REG_BG_CPSI_LOG_MS     15000U

#define REG_BACKOFF_START_MS  720000UL   // 12 Minuten
#define REG_BACKOFF_MAX_MS   2400000UL   // 40 Minuten (Deckel)
#define REG_MAX_ATTEMPTS      8U

#define REG_MANUAL_PLMN            "26201"   // Telekom DE
#define REG_MANUAL_AFTER_ATTEMPTS  1U

#define REG_FAST_TOGGLE_MS         3000UL
#define REG_CPSI_GRACE_MS          15000UL
#define REG_TOGGLE_MIN_BUDGET_MS    7000UL
#define REG_STUCK_THRESHOLD    5U

HardwareSerial simSerial(1);
HardwareSerial extSerial(2);

// ── State ─────────────────────────────────────────────────────────────────────
static char rxBuf[2048];
static char httpUrl[768];
static bool httpInitDone = false;
static bool cfunFull     = false;
static bool netRegistered = false;

enum RegState { REG_NOT_STARTED, REG_SEARCHING, REG_REGISTERED, REG_GAVE_UP };
static RegState regState = REG_NOT_STARTED;
static uint32_t regSearchStart = 0;
static uint32_t regBgLastPoll = 0;
static uint32_t regBgLastCpsiLog = 0;
static uint32_t regBackoffMs = REG_BACKOFF_START_MS;
static uint32_t regLastCopsTrigger = 0;
static uint8_t  regAttempts = 0;
static uint8_t  regConsecutiveFails = 0;
static uint32_t regLastFastToggle = 0;
static uint8_t  regFastToggleIdx  = 0;
static uint32_t regToggleGraceUntil = 0;
static bool     regGraceGranted     = false;

static char bodyBuf[2048];
static int  bodyLen = 0;
static int  lastDataLen = 0;
static int  lastHttpStatus = 0;

static char rawHttpReadBuf[2200];
static int  rawHttpReadLen = 0;

// FIX 46: Zeitpunkt der letzten gueltigen Modem-Antwort / URC
static uint32_t lastModemAliveMs = 0;

// FIX 47: letzter Befehl vom ext. µC + Neustart-Zaehler (ueberlebt Deep Sleep)
static uint32_t lastExtCmdMs = 0;
RTC_DATA_ATTR static uint32_t dailyRestartCount = 0;
static void dailyRestart(const char *reason);   // Vorwaertsdeklaration

// ══════════════════════════════════════════════════════════════════════════════
// FIX 48: Nichtfluechtiges Fehler-Log (LittleFS) + Netzzeit
// ══════════════════════════════════════════════════════════════════════════════
#define LOG_FILE            "/errlog.txt"
#define LOG_FILE_OLD        "/errlog_old.txt"
#define LOG_MAX_BYTES       65536UL      // danach Rotation
#define LOG_RATE_WINDOW_MS  600000UL     // 10 Min
#define LOG_RATE_MAX        40U          // max. Eintraege pro Fenster
#define TIME_RESYNC_MS      (6UL * 3600UL * 1000UL)

static bool     logReady = false;
static uint32_t bootCount = 0;
static uint32_t logWindowStart = 0;
static uint16_t logWindowCount = 0;
static uint32_t logSuppressed  = 0;

// Netzzeit (aus AT+CCLK, lokale Zeit laut Netz)
static bool     timeValid = false;
static uint32_t timeEpochBase = 0;
static uint32_t timeBaseMs = 0;
static uint32_t timeSyncMs = 0;
RTC_DATA_ATTR static uint32_t rtcEpochAfterSleep = 0;  // Zeit ueber Deep Sleep retten

// Tagesbilanz
static uint32_t statHttpOk = 0, statHttpFail = 0, statSapbrErr = 0;

static uint32_t nowEpoch() { return timeEpochBase + (millis() - timeBaseMs) / 1000UL; }

static int32_t daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y-399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;
    const unsigned doe = yoe * 365 + yoe/4 - yoe/100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

static void logEvent(const char *level, const char *fmt, ...) {
    char msg[200];
    va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof(msg), fmt, ap); va_end(ap);

    Serial.print("["); Serial.print(millis()); Serial.print("] [LOG-"); Serial.print(level);
    Serial.print("] "); Serial.println(msg);
    if (!logReady) return;

    // Rate-Limit (Flash-Schonung)
    if (millis() - logWindowStart > LOG_RATE_WINDOW_MS) {
        logWindowStart = millis(); logWindowCount = 0;
    }
    if (logWindowCount >= LOG_RATE_MAX) { logSuppressed++; return; }
    logWindowCount++;

    char tbuf[24] = "----------  --:--:--";
    if (timeValid) {
        time_t t = (time_t)nowEpoch(); struct tm tmv; gmtime_r(&t, &tmv);
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tmv);
    }
    char line[300];
    int n = snprintf(line, sizeof(line), "B%lu | %s | up %lus | %-4s | ",
                     (unsigned long)bootCount, tbuf, (unsigned long)(millis()/1000UL), level);
    if (logSuppressed > 0 && n < (int)sizeof(line)) {
        n += snprintf(line+n, sizeof(line)-n, "(%lu Eintraege unterdrueckt) ",
                      (unsigned long)logSuppressed);
        logSuppressed = 0;
    }
    if (n < (int)sizeof(line)) snprintf(line+n, sizeof(line)-n, "%s\n", msg);

    File f = LittleFS.open(LOG_FILE, "a");
    if (!f) return;
    if (f.size() > LOG_MAX_BYTES) {
        f.close();
        LittleFS.remove(LOG_FILE_OLD);
        LittleFS.rename(LOG_FILE, LOG_FILE_OLD);
        f = LittleFS.open(LOG_FILE, "a");
        if (!f) return;
    }
    f.print(line);
    f.close();
}

// Rohtext fuer Log entschaerfen (\r\n -> Leerzeichen, gekuerzt)
static const char* logSnippet(const char *in, int maxLen = 60) {
    static char out[64];
    int j = 0;
    for (int i = 0; in[i] && j < maxLen && j < (int)sizeof(out)-1; i++) {
        char c = in[i];
        if (c == '\r' || c == '\n') { if (j > 0 && out[j-1] != ' ') out[j++] = ' '; }
        else if (c >= 32 && c < 127) out[j++] = c;
    }
    out[j] = '\0';
    return out;
}

static void logInit() {
    Preferences prefs;
    if (prefs.begin("bridge", false)) {
        bootCount = prefs.getUInt("boot", 0) + 1;
        prefs.putUInt("boot", bootCount);
        prefs.end();
    }
    logReady = LittleFS.begin(true);   // true = beim ersten Mal formatieren
    Serial.print("[LOG] LittleFS "); Serial.println(logReady ? "bereit" : "FEHLER - kein Flash-Log!");
    logWindowStart = millis();
}

static void logPrintAll() {
    if (!logReady) { Serial.println("[LOG] Flash-Log nicht verfuegbar"); return; }
    const char *files[2] = { LOG_FILE_OLD, LOG_FILE };
    Serial.println("\n======== FEHLER-LOG (Flash) ========");
    for (int k = 0; k < 2; k++) {
        File f = LittleFS.open(files[k], "r");
        if (!f) continue;
        Serial.print("---- "); Serial.print(files[k]); Serial.print(" (");
        Serial.print(f.size()); Serial.println(" B) ----");
        uint8_t buf[128];
        while (f.available()) {
            int r = f.read(buf, sizeof(buf));
            Serial.write(buf, r);
            esp_task_wdt_reset();
        }
        f.close();
    }
    Serial.print("======== Ende (Flash belegt: ");
    Serial.print(LittleFS.usedBytes()); Serial.print(" / ");
    Serial.print(LittleFS.totalBytes()); Serial.println(" B) ========\n");
}

static void logClear() {
    if (!logReady) return;
    LittleFS.remove(LOG_FILE);
    LittleFS.remove(LOG_FILE_OLD);
    Serial.println("[LOG] Fehler-Log geloescht");
    logEvent("INFO", "Log geloescht (USB LOGCLR)");
}

// AT-Fehler loggen – erwartete/harmlose Befehle auslassen
static bool atLogSuppressed(const char *cmd) {
    return strcmp(cmd, "AT") == 0 ||                 // Boot-/Health-Ping (eigener Eintrag)
           strncmp(cmd, "AT+CRSM=214", 11) == 0 ||   // FPLMN-Schreiben oft verweigert
           strncmp(cmd, "AT+CSCLK", 8) == 0 ||
           strncmp(cmd, "AT+CPSMS", 8) == 0 ||
           strncmp(cmd, "AT+CEDRXS", 9) == 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 46: Watchdog
// ══════════════════════════════════════════════════════════════════════════════
// Ersetzt yield() in allen (zeitlich begrenzten) Warteschleifen: fuettert den
// Task-Watchdog, damit legitime lange Vorgaenge (SCAN, waitNetReg, HTTP) keinen
// Reset ausloesen. Ein echter Haenger ausserhalb dieser Schleifen -> Reset.
static inline void wdtYield() {
    esp_task_wdt_reset();
    yield();
}

static void wdtInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = HANG_RESTART_MS,
        .idle_core_mask = 0,          // Idle-Tasks nicht ueberwachen
        .trigger_panic  = true,       // Timeout -> Reset
    };
    if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) {
        esp_task_wdt_init(&cfg);
    }
#else
    esp_task_wdt_init(HANG_RESTART_S, true);   // IDF 4.x: init bzw. update
#endif
    esp_task_wdt_add(NULL);                     // loop()-Task ueberwachen
}

static void hangRestart(const char *reason) {
    Serial.print("\n["); Serial.print(millis()); Serial.print("] [HANG] ");
    Serial.print(reason);
    Serial.println(" -> Neustart der Bridge (ESP.restart)");
    logEvent("ERR", "Haenger-Neustart: %s", reason);   // FIX 48
    Serial.flush();
    led.setPixelColor(0, led.Color(80,0,0)); led.show();
    delay(500);
    ESP.restart();
}

// ══════════════════════════════════════════════════════════════════════════════
// Debug
// ══════════════════════════════════════════════════════════════════════════════
static void ts() { Serial.print('['); Serial.print(millis()); Serial.print("] "); }

static void printEscaped(const char *s, int maxLen = 200) {
    for (int i = 0; i < maxLen && s[i]; i++) {
        char c = s[i];
        if      (c == '\r') Serial.print("\\r");
        else if (c == '\n') Serial.print("\\n");
        else if (c < 32)  { Serial.print("\\x"); if(c<16) Serial.print('0'); Serial.print((uint8_t)c,HEX); }
        else                Serial.print(c);
    }
}

// FIX 39/41: xx-Payload aus rawHttpReadBuf nach gprs.lua-Regel parsen & loggen.
static void logParsedPayload(const char *raw, int len) {
    const char *p = strstr(raw, "xx");
    if (!p) {
        ts(); Serial.println("[PARSE] kein \"xx\"-Marker im Rohtext gefunden");
        return;
    }
    int off = (int)(p - raw);
    int avail = len - off;

    ts(); Serial.println("[PARSE] ---- xx-Payload (wie gprs.lua) ----");

    if (avail > 5) {
        char pd[5]; memcpy(pd, p+2, 4); pd[4] = '\0';
        ts(); Serial.print("[PARSE] pd     = "); Serial.print(pd);
        Serial.println("  (Pulldown-Code)");
    } else { ts(); Serial.println("[PARSE] pd     = <zu kurz>"); }

    if (avail > 9) {
        char nds[5]; memcpy(nds, p+6, 4); nds[4] = '\0';
        ts(); Serial.print("[PARSE] nds    = "); Serial.print(nds);
        Serial.print("  (Intervall/s = "); Serial.print(atoi(nds)); Serial.println(")");
    } else { ts(); Serial.println("[PARSE] nds    = <zu kurz>"); }

    if (avail > 10) {
        char reset = p[10];
        const char *meaning =
            (reset=='0') ? "keine Aktivitaet" :
            (reset=='1') ? "Remote-Reset" :
            (reset=='2') ? "Deepsleep" :
            (reset=='9') ? "Feuchte-Alarm (Rel.3 ein)" : "unbekannt";
        ts(); Serial.print("[PARSE] reset  = '");
        printEscaped(&reset, 1);
        Serial.print("'  (byte "); Serial.print((int)(uint8_t)reset);
        Serial.print(", "); Serial.print(meaning); Serial.println(")");
    } else { ts(); Serial.println("[PARSE] reset  = <zu kurz>"); }

    if (avail > 13) {
        char sensor[4]; memcpy(sensor, p+11, 3); sensor[3] = '\0';
        ts(); Serial.print("[PARSE] sensor = "); Serial.print(sensor);
        Serial.print("  (Anzahl = "); Serial.print(atoi(sensor)); Serial.println(")");
    } else { ts(); Serial.println("[PARSE] sensor = <zu kurz>"); }

    {
        const char *plStart = p + 2;
        int plLen = 0;
        while (plStart[plLen] && plStart[plLen] != '\r' && plStart[plLen] != '\n'
               && (plStart + plLen) < (raw + len)) plLen++;
        ts(); Serial.print("[PARSE] pl     = ");
        printEscaped(plStart, plLen);
        Serial.println("  (Nutzteil ab xx+2)");
    }

    ts(); Serial.println("[PARSE] ------------------------------------");
}

// ══════════════════════════════════════════════════════════════════════════════
// µC-Kommunikation
// ══════════════════════════════════════════════════════════════════════════════
static void extReply(const char *msg) {
    extSerial.print(msg); extSerial.print("\r\n");
    ts(); Serial.print("  ╔══ TX→µC ══╗ \""); printEscaped(msg); Serial.println("\\r\\n\"");
}

static void extReplyURC(const char *urc) {
    extSerial.print(urc); extSerial.print("\r\n\r\n"); extSerial.print("OK\r\n");
    ts(); Serial.print("  ╔══ URC→µC ══╗ \""); printEscaped(urc); Serial.println("\"");
}

// FIX 28: seit v26.11 nirgends mehr aufgerufen (Auto-Push entfernt).
static void extSendBodyURC(const char *body, int len) __attribute__((unused));
static void extSendBodyURC(const char *body, int len) {
    char hdr[32]; snprintf(hdr, sizeof(hdr), "+HTTPREAD: %d", len);
    extSerial.print(hdr); extSerial.print("\r\n");
    if (len > 0) extSerial.write((const uint8_t*)body, len);
    extSerial.print("\r\nOK\r\n");
    ts(); Serial.print("  ╔══ HTTPREAD→µC (auto) ══╗ "); Serial.print(len); Serial.println("B");
}

static void extSendFailURC(int httpStatus) {
    char urc[32]; snprintf(urc, sizeof(urc), "+SEND_FAIL: %d", httpStatus);
    extReplyURC(urc);
}

// ══════════════════════════════════════════════════════════════════════════════
// AT-Kern
// ══════════════════════════════════════════════════════════════════════════════
static bool sendAT(const char *cmd, const char *expected,
                   uint32_t timeout_ms, uint32_t drain_ms = 50)
{
    memset(rxBuf, 0, sizeof(rxBuf));
    uint16_t idx = 0; bool found = false;
    simSerial.print(cmd); simSerial.print("\r\n");
    ts(); Serial.print("[→SIM] "); Serial.println(cmd);

    uint32_t t0 = millis();
    while ((millis()-t0) < timeout_ms) {
        while (simSerial.available()) {
            char c = (char)simSerial.read();
            if (idx < sizeof(rxBuf)-1) rxBuf[idx++] = c; rxBuf[idx] = '\0';
        }
        if (!found) {
            if (strstr(rxBuf, expected)) found = true;
            if (strcmp(expected,"ERROR")!=0 && strstr(rxBuf,"ERROR")) {
                lastModemAliveMs = millis();   // FIX 46: Modem antwortet (ERROR ist auch Leben)
                ts(); Serial.print("[ERROR] "); Serial.print(cmd);
                Serial.print("  RESP: "); printEscaped(rxBuf,120); Serial.println();
                if (!atLogSuppressed(cmd)) logEvent("ERR", "AT-ERROR %s -> %s", cmd, logSnippet(rxBuf));
                return false;
            }
        } else {
            uint32_t td = millis();
            while ((millis()-td) < drain_ms) {
                while (simSerial.available()) {
                    char c = (char)simSerial.read();
                    if (idx < sizeof(rxBuf)-1) rxBuf[idx++] = c; rxBuf[idx] = '\0';
                }
                wdtYield();
            }
            lastModemAliveMs = millis();       // FIX 46
            ts(); Serial.print("[←SIM] "); printEscaped(rxBuf, 160); Serial.println();
            return true;
        }
        wdtYield();
    }
    ts(); Serial.print("[TIMEOUT] "); Serial.print(cmd);
    Serial.print("  PARTIAL: "); printEscaped(rxBuf,120); Serial.println();
    if (!atLogSuppressed(cmd)) logEvent("ERR", "AT-TIMEOUT %s (%lums)", cmd, (unsigned long)timeout_ms);
    return false;
}

// ══════════════════════════════════════════════════════════════════════════════
// NeoPixel
// ══════════════════════════════════════════════════════════════════════════════
static void ledSet(uint8_t r, uint8_t g, uint8_t b, uint32_t dur=0) {
    led.setPixelColor(0, led.Color(r,g,b)); led.show();
    if (dur > 0) { uint32_t t=millis(); while(millis()-t<dur) wdtYield(); led.setPixelColor(0,0); led.show(); }
}
static void flashGreen() { ledSet(0,80,0,300); }
static void flashRed()   { ledSet(80,0,0,500); }
static void ledBlue()    { ledSet(0,0,60); }
static void ledYellow()  { ledSet(70,50,0); }
static void ledOff()     { ledSet(0,0,0); }

// ══════════════════════════════════════════════════════════════════════════════
// CEREG stat parsen
// ══════════════════════════════════════════════════════════════════════════════
static int parseCeregStat(const char *buf) {
    const char *p = strstr(buf, "+CEREG:");
    if (!p) return -1;
    int n=0, stat=-1;
    if (sscanf(p, "+CEREG: %d,%d", &n, &stat)==2 ||
        sscanf(p, "+CEREG:%d,%d",  &n, &stat)==2) return stat;
    return -1;
}

static const char* statName(int s) {
    switch(s) {
        case 0: return "nicht registriert, sucht NICHT";
        case 1: return "REGISTRIERT (Home)";
        case 2: return "sucht...";
        case 3: return "ABGELEHNT";
        case 5: return "REGISTRIERT (Roaming)";
        default: return "?";
    }
}

static int readCsqRssi() {
    if (sendAT("AT+CSQ","OK",T_SHORT)) {
        int rssi=99,ber=99;
        char *p=strstr(rxBuf,"+CSQ:");
        if (p) sscanf(p,"+CSQ: %d,%d",&rssi,&ber);
        return rssi;
    }
    return 99;
}

static int csqToDbm(int rssi) {
    if (rssi < 0 || rssi > 31) return 0;
    return -113 + 2*rssi;
}

static void logCsqDbm(int rssi) __attribute__((unused));
static void logCsqDbm(int rssi) {
    ts(); Serial.print("[CSQ] rssi="); Serial.print(rssi);
    if (rssi==99) {
        Serial.println("  >>> KEIN SIGNAL <<<");
    } else {
        Serial.print("  ("); Serial.print(csqToDbm(rssi)); Serial.println(" dBm)");
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// Boot-Diagnose
// ══════════════════════════════════════════════════════════════════════════════
static void bootDiag() {
    Serial.println("\n──────── BOOT-DIAG ────────");
    sendAT("ATI","OK",T_SHORT);
    sendAT("AT+CPIN?","OK",T_SHORT);
    sendAT("AT+CIMI","OK",T_SHORT);
    int rssi = readCsqRssi();
    if (rssi==99) Serial.println("    >>> KEIN SIGNAL (CSQ=99) – Antenne/Standort prüfen <<<");
    else          { ts(); Serial.printf("    >>> RSSI=%d (~%d dBm) <<<\n", rssi, -113+2*rssi); }
    sendAT("AT+CGDCONT?","OK",T_SHORT);
    Serial.println("───────────────────────────\n");
}

// ══════════════════════════════════════════════════════════════════════════════
// Auf +CPIN: READY warten
// ══════════════════════════════════════════════════════════════════════════════
static bool waitSimReady(uint32_t timeout_ms = 15000) {
    ts(); Serial.println("[SIM] Warte auf CPIN: READY...");
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        {
            char tmp[256] = {0}; uint16_t ti = 0;
            uint32_t tw = millis();
            while (millis() - tw < 500) {
                while (simSerial.available()) {
                    char c = (char)simSerial.read();
                    if (ti < sizeof(tmp)-1) tmp[ti++] = c; tmp[ti] = '\0';
                }
                if (strstr(tmp, "READY") || strstr(tmp, "SIM Ready")) {
                    ts(); Serial.println("[SIM] READY (URC)");
                    return true;
                }
                wdtYield();
            }
        }
        if (sendAT("AT+CPIN?","OK",T_SHORT)) {
            if (strstr(rxBuf,"READY")) { ts(); Serial.println("[SIM] READY"); return true; }
        }
        delay(500);
    }
    ts(); Serial.println("[WARN] SIM nicht bereit – trotzdem weiter");
    logEvent("WARN", "SIM nicht bereit (kein CPIN READY)");
    return false;
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 22: Forbidden-PLMN-Liste (EF_FPLMN) zuruecksetzen
// ══════════════════════════════════════════════════════════════════════════════
static void clearFplmn() {
    ts(); Serial.println("[FPLMN] Lese aktuelle Sperrliste...");
    if (sendAT("AT+CRSM=176,28539,0,0,12","OK",T_SHORT)) {
        ts(); Serial.print("[FPLMN] Vorher: "); printEscaped(rxBuf,100); Serial.println();
    }
    ts(); Serial.println("[FPLMN] Setze zurueck (12x FF)...");
    bool ok = sendAT("AT+CRSM=214,28539,0,0,12,\"FFFFFFFFFFFFFFFFFFFFFFFF\"","OK",T_MEDIUM);
    ts();
    if (ok) Serial.println("[FPLMN] Zurueckgesetzt.");
    else    Serial.println("[FPLMN] Schreibzugriff verweigert (ERROR) - ignoriert.");
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 45: Alle Modem-Stromsparmodi abschalten (Modem bleibt wach)
// ══════════════════════════════════════════════════════════════════════════════
static void disableModemSleep() {
    ts(); Serial.println("[NOSLEEP] Modem-Stromsparmodi abschalten (CSCLK/CPSMS/CEDRXS)...");
    sendAT("AT+CSCLK=0","OK",T_SHORT);    // kein UART-/Slow-Clock-Sleep
    sendAT("AT+CPSMS=0","OK",T_SHORT);    // kein Power Saving Mode
    sendAT("AT+CEDRXS=0","OK",T_SHORT);   // kein eDRX
}

// ══════════════════════════════════════════════════════════════════════════════
// Modem initialisieren
// ══════════════════════════════════════════════════════════════════════════════
static void ensureModemOn() {
    for (int i = 0; i < 10; i++) {
        if (sendAT("AT","OK",T_AT)) { ts(); Serial.println("[OK] Modem aktiv"); goto modem_ok; }
        delay(500);
    }
    ts(); Serial.println("[INFO] Power-Key Puls");
    logEvent("WARN", "Modem antwortet nicht auf AT - Power-Key-Puls");
    pinMode(SIM_PWR_PIN, OUTPUT);
    digitalWrite(SIM_PWR_PIN, HIGH); delay(200); digitalWrite(SIM_PWR_PIN, LOW);
    for (int i = 0; i < 30; i++) {
        if (sendAT("AT","OK",T_AT)) { ts(); Serial.println("[OK] Modem nach Power-Key"); goto modem_ok; }
        delay(300);
    }
    // FIX 47: zweiter, langer Puls (Modem war per CPOWD komplett aus)
    ts(); Serial.println("[INFO] Power-Key Puls lang (1s)");
    digitalWrite(SIM_PWR_PIN, HIGH); delay(1000); digitalWrite(SIM_PWR_PIN, LOW);
    for (int i = 0; i < 30; i++) {
        if (sendAT("AT","OK",T_AT)) { ts(); Serial.println("[OK] Modem nach langem Power-Key"); goto modem_ok; }
        delay(300);
    }
    ts(); Serial.println("[ERROR] Modem antwortet nicht!");
    logEvent("ERR", "Modem tot: keine Antwort auch nach 2x Power-Key");
    return;

modem_ok:
    sendAT("ATE0","OK",T_SHORT);
    disableModemSleep();   // FIX 45

    cfunFull = sendAT("AT+CFUN?","OK",T_SHORT) && strstr(rxBuf,"+CFUN: 1");
    ts(); Serial.print("[CFUN] "); Serial.println(cfunFull ? "1 (ok)" : "0");

    if (!cfunFull) {
        ledBlue();
        ts(); Serial.println("[INIT] CFUN=1...");
        if (sendAT("AT+CFUN=1","OK",T_CFUN,500)) {
            cfunFull = true;
            ts(); Serial.println("[INIT] CFUN=1 OK");
        }
    }

    waitSimReady(20000);
    clearFplmn();
    bootDiag();
}

// ══════════════════════════════════════════════════════════════════════════════
// APN setzen – FIX 17: auf Kontext 0 UND 1
// ══════════════════════════════════════════════════════════════════════════════
static void setApn() {
    char buf[80];
    snprintf(buf, sizeof(buf), "AT+CGDCONT=%d,\"IP\",\"%s\"", PDP_CID, APN);
    sendAT(buf, "OK", T_SHORT);
    snprintf(buf, sizeof(buf), "AT+CGDCONT=1,\"IP\",\"%s\"", APN);
    sendAT(buf, "OK", T_SHORT);
    ts(); Serial.print("[APN] "); Serial.print(APN);
    Serial.print(" auf CID "); Serial.print(PDP_CID); Serial.println(" + 1");
}

// ══════════════════════════════════════════════════════════════════════════════
// Prüfen ob IP auf PDP_CID vergeben ist
// ══════════════════════════════════════════════════════════════════════════════
static bool hasIpOnContext() {
    if (!sendAT("AT+CGPADDR","OK",T_SHORT)) return false;
    char needle[20];
    snprintf(needle, sizeof(needle), "+CGPADDR: %d,", PDP_CID);
    char *p = strstr(rxBuf, needle);
    if (!p) return false;
    p += strlen(needle);
    if (*p == '"') p++;
    return (*p >= '1' && *p <= '9');
}

// ══════════════════════════════════════════════════════════════════════════════
// Auf Registrierung warten (gemeinsame Poll-Schleife)
// ══════════════════════════════════════════════════════════════════════════════
static int pollRegister(uint32_t timeout_ms) {
    uint32_t t0=millis(); int last=-99;
    while (millis()-t0 < timeout_ms) {
        if (sendAT("AT+CEREG?","OK",T_SHORT)) {
            int s=parseCeregStat(rxBuf);
            if (s!=last) {
                ts(); Serial.print("[REG] stat="); Serial.print(s);
                Serial.print(" ("); Serial.print(statName(s)); Serial.println(")");
                last=s;
            }
            if (s==1||s==5) return s;
            if (s==3)       return 3;
            if (s==2) {
                int r=readCsqRssi();
                ts(); Serial.print("[REG]   CSQ="); Serial.print(r);
                if (r==99) Serial.println(" (kein Signal!)");
                else { Serial.print(" (≈"); Serial.print(csqToDbm(r)); Serial.println(" dBm)"); }
            }
        }
        char tmp[256]={0}; uint16_t ti=0; uint32_t tw=millis();
        while (millis()-tw < 3000) {
            while (simSerial.available()) {
                char c=(char)simSerial.read();
                if (ti<sizeof(tmp)-1) tmp[ti++]=c; tmp[ti]='\0';
            }
            int s=parseCeregStat(tmp);
            if (s==1||s==5) return s;
            wdtYield();
        }
    }
    return -1;
}

// ══════════════════════════════════════════════════════════════════════════════
// Netzregistrierung mit Operator-Fallback (FIX 18) - nur manueller "REG"
// ══════════════════════════════════════════════════════════════════════════════
static bool waitNetReg() {
    ledBlue();
    sendAT("AT+CEREG=2","OK",T_SHORT);

    ts(); Serial.println("[NETREG] Versuch 1: Auto (COPS=0)...");
    sendAT("AT+COPS=0","OK",T_CFUN);

    int r = pollRegister(T_NETREG);
    if (r==1 || r==5) {
        sendAT("AT+CSQ","OK",T_SHORT);
        sendAT("AT+COPS?","OK",T_SHORT);
        ledOff(); netRegistered=true; return true;
    }
    if (r==3) { ts(); Serial.println("[NETREG] Abgelehnt – SIM/Tarif prüfen"); ledYellow(); return false; }

    ts(); Serial.println("[NETREG] Auto fehlgeschlagen – probiere Betreiber manuell...");
    ledYellow();

    for (int i=0; i<NUM_OPERATORS; i++) {
        Serial.println();
        ts(); Serial.print("[NETREG] === Versuch Betreiber: ");
        Serial.print(DE_OPERATORS[i].name); Serial.print(" (");
        Serial.print(DE_OPERATORS[i].plmn); Serial.println(") ===");

        char cmd[40];
        snprintf(cmd, sizeof(cmd), "AT+COPS=1,2,\"%s\"", DE_OPERATORS[i].plmn);
        sendAT(cmd, "OK", T_CFUN);
        delay(2000);

        int rr = pollRegister(T_OPREG);
        if (rr==1 || rr==5) {
            ts(); Serial.print("[NETREG] ✓✓✓ REGISTRIERT über ");
            Serial.print(DE_OPERATORS[i].name); Serial.println(" ✓✓✓");
            sendAT("AT+CSQ","OK",T_SHORT);
            sendAT("AT+COPS?","OK",T_SHORT);
            ledOff(); netRegistered=true; return true;
        }
        ts(); Serial.print("[NETREG] ");
        Serial.print(DE_OPERATORS[i].name); Serial.println(" fehlgeschlagen, nächster...");
    }

    sendAT("AT+COPS=0","OK",T_CFUN);
    ts(); Serial.println("[NETREG] Alle Betreiber fehlgeschlagen – Antenne/Standort prüfen");
    ledYellow(); return false;
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 23: Nicht-blockierende Hintergrund-Registrierung
// ══════════════════════════════════════════════════════════════════════════════
static void startRegistrationIfNeeded() {
    if (regState != REG_NOT_STARTED) return;

    if (!cfunFull) {
        ledBlue();
        if (sendAT("AT+CFUN=1","OK",T_CFUN,200)) cfunFull = true;
    }
    sendAT("AT+CEREG=2","OK",T_SHORT);
    ts(); Serial.println("[NETREG-BG] Start: AT+COPS=0 Trigger...");
    sendAT("AT+COPS=0","OK",T_MEDIUM);

    regState = REG_SEARCHING;
    regSearchStart = millis();
    regBgLastPoll = millis();
    regBgLastCpsiLog = millis();
    regBackoffMs = REG_BACKOFF_START_MS;
    regLastCopsTrigger = millis();
    regAttempts = 0;
    regConsecutiveFails = 0;
    regLastFastToggle = millis();
    regFastToggleIdx = 0;
    regToggleGraceUntil = 0;
    regGraceGranted = false;
    ledYellow();
}

static void logCpsiBg() {
    if (sendAT("AT+CPSI?","OK",T_SHORT)) {
        const char *p = strstr(rxBuf, "+CPSI:");
        ts(); Serial.print("[NETREG-BG][CPSI] ");
        printEscaped(p ? p : rxBuf, 100);
        Serial.println();
    }
}

// FIX 48: erfolgreiche Registrierung loggen (rxBuf enthaelt COPS?-Antwort)
static void logRegistered() {
    const char *p = strstr(rxBuf, "+COPS:");
    logEvent("INFO", "Netz registriert nach %lus: %s",
             (unsigned long)((millis()-regSearchStart)/1000UL),
             p ? logSnippet(p, 40) : "?");
}

static void advanceBackoff(const char *reason) {
    regAttempts++;
    if (regAttempts >= REG_MAX_ATTEMPTS) {
        logEvent("ERR", "Registrierung AUFGEGEBEN nach %u Zyklen (%s)", regAttempts, reason);
        ts(); Serial.print("[NETREG-BG] Aufgegeben nach ");
        Serial.print(regAttempts);
        Serial.print(" Backoff-Zyklen (");
        Serial.print(reason);
        Serial.println("). Ruhezustand - 'REG' oder Neustart zum Neubeginn.");
        regState = REG_GAVE_UP;
        ledYellow();
        return;
    }
    logEvent("WARN", "Registrierung Backoff #%u (%s), naechster Versuch in %lu Min",
             regAttempts, reason, (unsigned long)(regBackoffMs/60000UL));
    ts(); Serial.print("[NETREG-BG] Backoff (");
    Serial.print(reason);
    Serial.print("): naechster Versuch #");
    Serial.print(regAttempts + 1);
    Serial.print(" in ");
    Serial.print(regBackoffMs / 60000UL);
    Serial.println(" Min (1NCE No-Harm).");
    regLastCopsTrigger = millis();
    regBackoffMs *= 2;
    if (regBackoffMs > REG_BACKOFF_MAX_MS) regBackoffMs = REG_BACKOFF_MAX_MS;
}

static void pollRegistrationBackground(uint32_t callerBudgetMs = 0xFFFFFFFFUL) {
    if (regState != REG_SEARCHING) return;

    if (millis() - regLastFastToggle > REG_FAST_TOGGLE_MS) {
        regLastFastToggle = millis();

        if (sendAT("AT+CEREG?","OK",T_QUICK)) {
            int s = parseCeregStat(rxBuf);
            if (s==1 || s==5) {
                ts(); Serial.println("[NETREG-BG] Bereits registriert - "
                                      "Fast-Toggle wird NICHT gesendet.");
                sendAT("AT+CSQ","OK",T_QUICK);
                sendAT("AT+COPS?","OK",T_QUICK);
                regState = REG_REGISTERED; logRegistered();
                netRegistered = true;
                ledOff();
                return;
            }
        }

        if (callerBudgetMs < REG_TOGGLE_MIN_BUDGET_MS) {
            ts(); Serial.print("[NETREG-BG] Zu wenig Zeitbudget (");
            Serial.print(callerBudgetMs); Serial.println("ms) fuer Toggle - "
                          "uebersprungen, naechster Versuch aus loop().");
            return;
        }

        if (millis() < regToggleGraceUntil) {
            ts(); Serial.print("[NETREG-BG] Gnadenfrist aktiv (noch ");
            Serial.print((regToggleGraceUntil - millis())/1000);
            Serial.println("s) - kein Toggle, Attach laeuft...");
            return;
        }

        if (!regGraceGranted && sendAT("AT+CPSI?","OK",T_QUICK)) {
            if (strstr(rxBuf, ",Online,")) {
                regGraceGranted = true;
                regToggleGraceUntil = millis() + REG_CPSI_GRACE_MS;
                ts(); Serial.print("[NETREG-BG] CPSI zeigt Online-Zelle - Gnadenfrist ");
                Serial.print(REG_CPSI_GRACE_MS/1000);
                Serial.println("s, kein Toggle jetzt.");
                return;
            }
        }

        const Operator &op = DE_OPERATORS[regFastToggleIdx];
        regFastToggleIdx = (regFastToggleIdx + 1) % NUM_OPERATORS;
        regGraceGranted = false;
        char copsCmd[40];
        snprintf(copsCmd, sizeof(copsCmd), "AT+COPS=1,2,\"%s\"", op.plmn);
        ts(); Serial.print("[NETREG-BG] Fast-Toggle(3s) → ");
        Serial.print(op.name); Serial.print(" "); Serial.print(copsCmd);
        Serial.println(" ...");
        sendAT(copsCmd, "OK", T_QUICK);
    }

    if (millis() - regBgLastPoll < REG_BG_POLL_MS) return;
    regBgLastPoll = millis();

    if (sendAT("AT+CEREG?","OK",T_QUICK)) {
        regConsecutiveFails = 0;
        int s = parseCeregStat(rxBuf);
        ts(); Serial.print("[NETREG-BG] stat="); Serial.print(s);
        Serial.print(" ("); Serial.print((millis()-regSearchStart)/1000);
        Serial.println("s seit Start)");

        if (s==1 || s==5) {
            ts(); Serial.println("[NETREG-BG] ✓ REGISTRIERT");
            sendAT("AT+CSQ","OK",T_SHORT);
            sendAT("AT+COPS?","OK",T_SHORT);
            regState = REG_REGISTERED; logRegistered();
            netRegistered = true;
            ledOff();
            return;
        }
        if (s==3) {
            advanceBackoff("stat=3 abgelehnt");
            return;
        }
    } else {
        regConsecutiveFails++;
        ts(); Serial.print("[NETREG-BG] AT+CEREG? keine Antwort (");
        Serial.print(regConsecutiveFails); Serial.println("x in Folge)");
        if (regConsecutiveFails == REG_STUCK_THRESHOLD)
            logEvent("ERR", "Modem haengt: %u x keine Antwort auf AT+CEREG?", REG_STUCK_THRESHOLD);
        if (regConsecutiveFails >= REG_STUCK_THRESHOLD) {
            ts(); Serial.println("[NETREG-BG] ⚠ MODEM HAENGT FEST - serieller Port "
                                  "antwortet nicht. FIX 46: Neustart erfolgt "
                                  "automatisch, wenn das Modem 5 Min stumm bleibt.");
            regConsecutiveFails = REG_STUCK_THRESHOLD;
        }
    }

    if (millis() - regBgLastCpsiLog > REG_BG_CPSI_LOG_MS) {
        regBgLastCpsiLog = millis();
        logCpsiBg();
    }

    if (millis() - regLastCopsTrigger > regBackoffMs) {
        if (regAttempts >= REG_MANUAL_AFTER_ATTEMPTS) {
            char copsCmd[40];
            snprintf(copsCmd, sizeof(copsCmd), "AT+COPS=1,2,\"%s\"", REG_MANUAL_PLMN);
            ts(); Serial.print("[NETREG-BG] Backoff abgelaufen - manueller ");
            Serial.print(copsCmd); Serial.print(" (Versuch #");
            Serial.print(regAttempts + 1); Serial.println(")...");
            sendAT(copsCmd, "OK", T_MEDIUM);
        } else {
            ts(); Serial.print("[NETREG-BG] Backoff abgelaufen - erneuter "
                                "AT+COPS=0 (Versuch #");
            Serial.print(regAttempts + 1); Serial.println(")...");
            sendAT("AT+COPS=0","OK",T_MEDIUM);
        }
        advanceBackoff("kein Fortschritt");
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// HTTP GET – FIX 17: Kontext 0
// ══════════════════════════════════════════════════════════════════════════════
static String encodeDataParam(const String &s) {
    String out; out.reserve(s.length()+32);
    for (size_t i=0; i<s.length(); i++) {
        char c=s[i];
        if      (c==' ') out+="%20";
        else if (c==';') out+="%3B";
        else             out+=c;
    }
    return out;
}

static String buildFullUrl(const char *url) {
    String u(url);
    if (!u.startsWith("http://") && !u.startsWith("https://")) u="http://"+u;
    int q=u.indexOf('?'); if (q<0) return u;
    String base=u.substring(0,q+1), query=u.substring(q+1);
    int d=query.indexOf("data="); if (d<0) return u;
    return base+query.substring(0,d+5)+encodeDataParam(query.substring(d+5));
}

static int sim7028HttpGet(const char *fullUrl) {
    if (!hasIpOnContext()) {
        ts(); Serial.print("[HTTP] Keine IP auf CID "); Serial.print(PDP_CID);
        Serial.println(" – PDP aktivieren");
        char actCmd[24];
        snprintf(actCmd, sizeof(actCmd), "AT+CGACT=1,%d", PDP_CID);
        sendAT(actCmd, "OK", 30000);
        delay(1000);
    }

    if (!sendAT("AT+HTTPINIT","OK",T_HTTP)) {
        Serial.println("[HTTP] HTTPINIT fehlgeschlagen");
        logEvent("ERR", "HTTPINIT fehlgeschlagen");
        return 0;
    }
    httpInitDone=true;

    char cidCmd[32];
    snprintf(cidCmd, sizeof(cidCmd), "AT+HTTPPARA=\"CID\",%d", PDP_CID);
    sendAT(cidCmd, "OK", T_SHORT);

    char urlCmd[800];
    snprintf(urlCmd,sizeof(urlCmd),"AT+HTTPPARA=\"URL\",\"%s\"",fullUrl);
    if (!sendAT(urlCmd,"OK",T_SHORT)) {
        sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; return 0;
    }

    memset(rxBuf,0,sizeof(rxBuf));
    simSerial.print("AT+HTTPACTION=0\r\n");
    ts(); Serial.println("[→SIM] AT+HTTPACTION=0");

    uint16_t idx=0; bool gotOK=false, gotAction=false;
    uint32_t t0=millis();
    while (millis()-t0 < T_HTTP) {
        while (simSerial.available()) {
            char c=(char)simSerial.read();
            if (idx<sizeof(rxBuf)-1) rxBuf[idx++]=c; rxBuf[idx]='\0';
        }
        if (!gotOK     && strstr(rxBuf,"OK"))           gotOK=true;
        if (!gotAction && strstr(rxBuf,"+HTTPACTION:")) gotAction=true;
        if (gotOK && gotAction) break;
        if (strstr(rxBuf,"+HTTP_NONET_EVENT")) {
            Serial.println("[HTTP] NONET_EVENT – Netz verloren");
            logEvent("ERR", "HTTP: Netz verloren (+HTTP_NONET_EVENT)");
            netRegistered=false;
            regState = REG_NOT_STARTED;   // FIX 45: beim naechsten SAPBR neu registrieren
            sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; return 0;
        }
        wdtYield();
    }
    if (!gotAction) {
        Serial.println("[HTTP] Kein HTTPACTION URC – Timeout");
        logEvent("ERR", "HTTP: kein +HTTPACTION nach %lus", (unsigned long)(T_HTTP/1000));
        sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; return 0;
    }
    lastModemAliveMs = millis();   // FIX 46

    int httpStatus=0, dataLen=0;
    char *ap=strstr(rxBuf,"+HTTPACTION:");
    if (ap) { int m=0; sscanf(ap,"+HTTPACTION: %d,%d,%d",&m,&httpStatus,&dataLen); }
    lastDataLen = dataLen;
    lastHttpStatus = httpStatus;
    ts(); Serial.print("[HTTP] Status="); Serial.print(httpStatus);
    Serial.print(" DataLen="); Serial.println(dataLen);

    memset(bodyBuf,0,sizeof(bodyBuf)); bodyLen=0;
    memset(rawHttpReadBuf,0,sizeof(rawHttpReadBuf)); rawHttpReadLen=0;

    if (dataLen > 0) {
        int readLen=min(dataLen,(int)(sizeof(bodyBuf)-1));

        {
            uint32_t tflush = millis();
            int flushed = 0;
            while (millis() - tflush < 100) {
                while (simSerial.available()) { simSerial.read(); flushed++; }
                wdtYield();
            }
            if (flushed > 0) {
                ts(); Serial.print("[HTTPREAD] Puffer-Reste verworfen: ");
                Serial.print(flushed); Serial.println("B (verhindert fruehen K-Trigger beim µC)");
            }
        }

        char readCmd[32]; snprintf(readCmd,sizeof(readCmd),"AT+HTTPREAD=0,%d",readLen);
        memset(rxBuf,0,sizeof(rxBuf)); uint16_t ridx=0;
        simSerial.print(readCmd); simSerial.print("\r\n");
        ts(); Serial.println(readCmd);

        uint32_t tr=millis();
        while (millis()-tr < T_MEDIUM) {
            while (simSerial.available()) {
                char c=(char)simSerial.read();
                if (ridx<sizeof(rxBuf)-1) rxBuf[ridx++]=c; rxBuf[ridx]='\0';
            }
            if (strstr(rxBuf,"+HTTPREAD: 0")) {
                uint32_t drainUntil = millis() + 150;
                while (millis() < drainUntil) {
                    while (simSerial.available()) {
                        char c=(char)simSerial.read();
                        if (ridx<sizeof(rxBuf)-1) rxBuf[ridx++]=c; rxBuf[ridx]='\0';
                    }
                    wdtYield();
                }
                break;
            }
            wdtYield();
        }

        char *rawStart = strstr(rxBuf, "+HTTPREAD:");
        if (!rawStart) rawStart = rxBuf;
        int rawLen = min((int)strlen(rawStart), (int)(sizeof(rawHttpReadBuf)-1));
        if (rawLen > 0) { memcpy(rawHttpReadBuf, rawStart, rawLen); rawHttpReadBuf[rawLen]='\0'; rawHttpReadLen=rawLen; }

        ts(); Serial.print("[HTTPREAD-RAW] "); Serial.print(rawLen); Serial.println("B, Originaltext:");
        printEscaped(rawHttpReadBuf, rawLen);
        Serial.println();
        logParsedPayload(rawHttpReadBuf, rawLen);

        char *ds=strstr(rxBuf,"+HTTPREAD:");
        if (ds) {
            char *nl=strchr(ds,'\n');
            if (nl) {
                nl++;
                char *em=strstr(nl,"+HTTPREAD: 0");
                int blen = em ? (int)(em-nl) : (int)strlen(nl);
                while (blen>0 && (nl[blen-1]=='\r'||nl[blen-1]=='\n')) blen--;
                blen=min(blen,(int)(sizeof(bodyBuf)-1));
                if (blen>0) { memcpy(bodyBuf,nl,blen); bodyBuf[blen]='\0'; bodyLen=blen; }
            }
        }
        ts(); Serial.print("[HTTP] Body "); Serial.print(bodyLen); Serial.println("B");
    }

    sendAT("AT+HTTPTERM","OK",T_SHORT);
    httpInitDone=false;
    return httpStatus;
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 19: Body LIVE vom Modem lesen
// ══════════════════════════════════════════════════════════════════════════════
static bool liveHttpRead(int dataLen) {
    if (dataLen <= 0) return false;
    int readLen = min(dataLen, (int)(sizeof(bodyBuf)-1));

    {
        uint32_t tflush = millis();
        int flushed = 0;
        while (millis() - tflush < 100) {
            while (simSerial.available()) { simSerial.read(); flushed++; }
            wdtYield();
        }
        if (flushed > 0) {
            ts(); Serial.print("[HTTPREAD-live] Puffer-Reste verworfen: ");
            Serial.print(flushed); Serial.println("B");
        }
    }

    char readCmd[32]; snprintf(readCmd, sizeof(readCmd), "AT+HTTPREAD=0,%d", readLen);
    memset(rxBuf, 0, sizeof(rxBuf)); uint16_t ridx=0;
    simSerial.print(readCmd); simSerial.print("\r\n");
    ts(); Serial.print("[→SIM] "); Serial.println(readCmd);

    uint32_t tr=millis();
    while (millis()-tr < T_MEDIUM) {
        while (simSerial.available()) {
            char c=(char)simSerial.read();
            if (ridx<sizeof(rxBuf)-1) rxBuf[ridx++]=c; rxBuf[ridx]='\0';
        }
        if (strstr(rxBuf,"+HTTPREAD: 0")) {
            uint32_t drainUntil = millis() + 150;
            while (millis() < drainUntil) {
                while (simSerial.available()) {
                    char c=(char)simSerial.read();
                    if (ridx<sizeof(rxBuf)-1) rxBuf[ridx++]=c; rxBuf[ridx]='\0';
                }
                wdtYield();
            }
            break;
        }
        wdtYield();
    }

    memset(rawHttpReadBuf,0,sizeof(rawHttpReadBuf)); rawHttpReadLen=0;
    char *rawStart = strstr(rxBuf, "+HTTPREAD:");
    if (!rawStart) rawStart = rxBuf;
    int rawLen = min((int)strlen(rawStart), (int)(sizeof(rawHttpReadBuf)-1));
    if (rawLen > 0) { memcpy(rawHttpReadBuf, rawStart, rawLen); rawHttpReadBuf[rawLen]='\0'; rawHttpReadLen=rawLen; }

    ts(); Serial.print("[HTTPREAD-RAW live] "); Serial.print(rawLen); Serial.println("B, Originaltext:");
    printEscaped(rawHttpReadBuf, rawLen);
    Serial.println();
    logParsedPayload(rawHttpReadBuf, rawLen);

    memset(bodyBuf,0,sizeof(bodyBuf)); bodyLen=0;
    char *ds=strstr(rxBuf,"+HTTPREAD:");
    if (ds) {
        char *nl=strchr(ds,'\n');
        if (nl) {
            nl++;
            char *em=strstr(nl,"+HTTPREAD: 0");
            int blen = em ? (int)(em-nl) : (int)strlen(nl);
            while (blen>0 && (nl[blen-1]=='\r'||nl[blen-1]=='\n')) blen--;
            blen=min(blen,(int)(sizeof(bodyBuf)-1));
            if (blen>0) { memcpy(bodyBuf,nl,blen); bodyBuf[blen]='\0'; bodyLen=blen; }
        }
    }
    ts(); Serial.print("[HTTPREAD-live] "); Serial.print(bodyLen); Serial.println("B");
    return bodyLen > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// CSQ Handler für µC
// ══════════════════════════════════════════════════════════════════════════════
static void handleCsq() {
    int rssi=99, ber=99;
    for (int i=0; i<3; i++) {
        if (sendAT("AT+CSQ","OK",T_SHORT,50)) {
            char *p=strstr(rxBuf,"+CSQ:");
            if (p) sscanf(p,"+CSQ: %d,%d",&rssi,&ber);
        }
        if (rssi!=99) break;
        delay(400);
    }
    char line[24]; snprintf(line,sizeof(line),"+CSQ: %d,%d",rssi,ber);
    ts(); Serial.print("[CSQ] "); Serial.println(line);
    if (rssi != 99) { ts(); Serial.print("[CSQ]   ≈ "); Serial.print(csqToDbm(rssi)); Serial.println(" dBm"); }
    extReplyURC(line);
}

// ══════════════════════════════════════════════════════════════════════════════
// Befehlsverarbeitung vom µC
// ══════════════════════════════════════════════════════════════════════════════
static bool isAtCommand(const String &l) {
    if (l.length()<2) return false;
    if ((uint8_t)l[0]>127||(uint8_t)l[1]>127) return false;
    return l.startsWith("AT");
}

static void handleExtCommand(const String &rawCmd) {
    String cmd=rawCmd; cmd.trim();
    if (cmd.length()==0) return;
    lastExtCmdMs = millis();   // FIX 47: µC aktiv -> taeglichen Neustart aufschieben
    if (!isAtCommand(cmd)) {
        ts(); Serial.print("[IGN] ");
        for (int i=0;i<min((int)cmd.length(),40);i++) { char c=cmd[i]; Serial.print((c>=32&&c<127)?c:'?'); }
        Serial.println(); return;
    }

    ts(); Serial.print("  ╚══ RX←µC ══╗ \""); printEscaped(cmd.c_str(),cmd.length()); Serial.println("\"");

    if (cmd=="AT")                                  { extReply("OK"); return; }
    if (cmd=="ATE0")                                { sendAT("ATE0","OK",T_SHORT); extReply("OK"); return; }
    if (cmd=="AT+CBC")                              { extReplyURC("+CBC: 0,100,4200"); return; }
    if (cmd=="AT+CSQ")                              { handleCsq(); return; }
    if (cmd.startsWith("AT+SAPBR=3,1") && cmd.indexOf("CONTYPE")>=0) { extReply("OK"); return; }
    if (cmd.startsWith("AT*MCGDEFCONT"))            { extReply("OK"); return; }
    if (cmd=="AT+HTTPSSL=0"||cmd=="AT+HTTPSSL=1")   { extReply("OK"); return; }

    if (cmd.startsWith("AT+SAPBR=3,1") && cmd.indexOf("APN")>=0) {
        setApn();
        extReply("OK"); return;
    }

    if (cmd=="AT+SAPBR=1,1") {
        if (regState == REG_GAVE_UP) {
            ts(); Serial.println("[SAPBR] Registrierung im Ruhezustand "
                                  "(Backoff aufgebraucht) - ERROR");
            statSapbrErr++;
            logEvent("ERR", "SAPBR ERROR: Registrierung aufgegeben (GAVE_UP)");
            extReply("ERROR");
            return;
        }
        startRegistrationIfNeeded();

        uint32_t waitStart = millis();
        while (millis() - waitStart < SAPBR_WAIT_MS) {
            uint32_t elapsed = millis() - waitStart;
            uint32_t remaining = (elapsed < SAPBR_WAIT_MS) ? (SAPBR_WAIT_MS - elapsed) : 0;
            pollRegistrationBackground(remaining);
            if (regState == REG_REGISTERED) break;
            delay(300);
            esp_task_wdt_reset();
        }

        if (regState == REG_REGISTERED) {
            ts(); Serial.println("[SAPBR] registriert - OK");
            extReply("OK");
        } else {
            ts(); Serial.print("[SAPBR] noch nicht registriert nach ");
            Serial.print(SAPBR_WAIT_MS/1000);
            Serial.println("s - ERROR (Hintergrundsuche laeuft weiter)");
            statSapbrErr++;
            logEvent("ERR", "SAPBR ERROR: nach %lus nicht registriert", (unsigned long)(SAPBR_WAIT_MS/1000));
            extReply("ERROR");
        }
        return;
    }

    if (cmd=="AT+HTTPINIT") {
        memset(bodyBuf,0,sizeof(bodyBuf)); bodyLen=0; lastDataLen=0;
        memset(rawHttpReadBuf,0,sizeof(rawHttpReadBuf)); rawHttpReadLen=0;
        memset(httpUrl,0,sizeof(httpUrl));
        if (httpInitDone) { sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; }
        extReply("OK"); return;
    }

    if (cmd.startsWith("AT+HTTPPARA=\"URL\"")) {
        int p=cmd.indexOf("\"URL\"");
        if (p>=0) {
            int open=cmd.indexOf('"',p+5), close=cmd.lastIndexOf('"');
            if (open>=0&&close>open) cmd.substring(open+1,close).toCharArray(httpUrl,sizeof(httpUrl));
        }
        ts(); Serial.print("[URL] "); Serial.println(httpUrl);
        extReply("OK"); return;
    }

    if (cmd=="AT+HTTPACTION=0") {
        // FIX 44: SYNCHRON – erst kompletter HTTP-Vorgang, DANN OK.
        if (strlen(httpUrl)==0) {
            logEvent("ERR", "HTTPACTION ohne URL (601)");
            statHttpFail++;
            extReply("OK");
            extSendFailURC(601);
            flashRed();
            return;
        }

        String fullUrl=buildFullUrl(httpUrl);
        ts(); Serial.print("[GET] "); Serial.println(fullUrl);

        uint32_t httpStart = millis();
        int status=sim7028HttpGet(fullUrl.c_str());
        uint32_t httpDur = millis() - httpStart;

        ts(); Serial.print("[HTTPACTION] status="); Serial.print(status);
        Serial.print(" len="); Serial.print(bodyLen);
        Serial.print(" dauer="); Serial.print(httpDur);
        Serial.println("ms (FIX 44: OK erst nach fertigem Read)");

        if (httpDur > (EXT_UC_HARD_TIMEOUT_MS - 2000)) {
            ts(); Serial.print("[HTTPACTION] ⚠ Dauer nahe µC-Timeout (");
            Serial.print(httpDur); Serial.print("ms von ");
            Serial.print(EXT_UC_HARD_TIMEOUT_MS);
            Serial.println("ms) - µC koennte das OK verpassen.");
            logEvent("WARN", "HTTP-Dauer %lums nahe µC-Timeout", (unsigned long)httpDur);
        }

        extReply("OK");

        if (status==200||status==302) {
            statHttpOk++;
            flashGreen();
        } else {
            statHttpFail++;
            logEvent("ERR", "HTTP fehlgeschlagen: status=%d dauer=%lums", status, (unsigned long)httpDur);
            char urc[64];
            snprintf(urc,sizeof(urc),"\r\n+HTTPACTION: 0,%d,%d\r\n",status,bodyLen);
            extSerial.print(urc);
            extSendFailURC(status);
            flashRed();
        }
        return;
    }

    if (cmd=="AT+HTTPREAD") {
        if (rawHttpReadLen==0 && lastDataLen>0) {
            ts(); Serial.println("[HTTPREAD] Cache leer – lese live vom Modem");
            liveHttpRead(lastDataLen);
        }
        if (rawHttpReadLen > 0) {
            ts(); Serial.print("[HTTPREAD→µC, Originaltext] "); Serial.print(rawHttpReadLen); Serial.println("B:");
            printEscaped(rawHttpReadBuf, rawHttpReadLen);
            Serial.println();
            // FIX 42/43: EXAKTES SIM800-Antwortformat (ein Block)
            {
                static char outBuf[2400];
                int outLen = 0;
                outLen += snprintf(outBuf + outLen, sizeof(outBuf) - outLen,
                                   "\r\n\r\n+HTTPACTION: 0,%d,%d\r\n\r\n",
                                   lastHttpStatus, lastDataLen);
                int readCopyLen = rawHttpReadLen;
                {
                    char *firstHdr = strstr(rawHttpReadBuf, "+HTTPREAD:");
                    char *trailer = NULL;
                    if (firstHdr) {
                        trailer = strstr(firstHdr + 10, "+HTTPREAD: 0");
                    }
                    if (trailer) {
                        readCopyLen = (int)(trailer - rawHttpReadBuf);
                        while (readCopyLen > 0 &&
                               (rawHttpReadBuf[readCopyLen-1]=='\r' ||
                                rawHttpReadBuf[readCopyLen-1]=='\n')) readCopyLen--;
                    }
                }
                if (outLen + readCopyLen < (int)sizeof(outBuf)) {
                    memcpy(outBuf + outLen, rawHttpReadBuf, readCopyLen);
                    outLen += readCopyLen;
                }
                const char tail[] = "\r\nOK\r\n";
                if (outLen + (int)(sizeof(tail)-1) < (int)sizeof(outBuf)) {
                    memcpy(outBuf + outLen, tail, sizeof(tail)-1);
                    outLen += sizeof(tail)-1;
                }
                extSerial.write((const uint8_t*)outBuf, outLen);
                ts(); Serial.print("[HTTPREAD→µC] SIM800-Format (FIX 43, Trailer entfernt), ");
                Serial.print(outLen); Serial.println("B:");
                printEscaped(outBuf, outLen);
                Serial.println();
            }
        } else {
            extSerial.print("+HTTPREAD: 0\r\n\r\nOK\r\n");
            ts(); Serial.println("[HTTPREAD→µC] 0B (kein Rohtext verfuegbar)");
            logEvent("WARN", "HTTPREAD: kein Payload fuer µC (lastDataLen=%d)", lastDataLen);
        }
        return;
    }

    if (cmd=="AT+HTTPTERM"||cmd=="AT+SAPBR=0,1") {
        if (httpInitDone) { sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; }
        extReply("OK"); return;
    }

    if (cmd=="AT+CPOWD=1") {
        if (httpInitDone) { sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; }
        // FIX 33/35: AT+CPOWD=1 NICHT ans Modem, kein 'K' in der Antwort.
        // FIX 45: Modem + Bridge bleiben wach und eingebucht -> cfunFull,
        //         netRegistered und regState werden NICHT mehr zurueckgesetzt.
        memset(bodyBuf,0,sizeof(bodyBuf)); bodyLen=0; lastDataLen=0;
        memset(rawHttpReadBuf,0,sizeof(rawHttpReadBuf)); rawHttpReadLen=0;
        extSerial.print("NORMAL POWER DOWN\r\n");
        ts(); Serial.println("[CPOWD] an µC: NORMAL POWER DOWN (Modem bleibt AN & eingebucht, Bridge bleibt wach - FIX 45)");
        return;
    }

    // Passthrough
    ts(); Serial.print("[PASS] "); Serial.println(cmd);
    simSerial.print(cmd); simSerial.print("\r\n");
    String resp; uint32_t tp=millis();
    while (millis()-tp<T_MEDIUM) {
        while (simSerial.available()) resp+=(char)simSerial.read();
        if (resp.indexOf("OK")>=0||resp.indexOf("ERROR")>=0) { lastModemAliveMs = millis(); break; }
        wdtYield();
    }
    extSerial.print(resp);
}

// ══════════════════════════════════════════════════════════════════════════════
// Ext. µC lesen
// ══════════════════════════════════════════════════════════════════════════════
static String extCmdBuf;
static void readExtSerial() {
    while (extSerial.available()) {
        char c=(char)extSerial.read();
        if (c=='\n') {
            if (extCmdBuf.length()>0) { handleExtCommand(extCmdBuf); extCmdBuf=""; }
        } else if (c!='\r') {
            extCmdBuf+=c;
            if (extCmdBuf.length()>1000) extCmdBuf="";
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// USB-Serial Eingabe (Diagnose + manuelle AT-Befehle)
// ══════════════════════════════════════════════════════════════════════════════
static String usbCmdBuf;
static void readUsbSerial() {
    while (Serial.available()) {
        char c=(char)Serial.read();
        if (c=='\n' || c=='\r') {
            if (usbCmdBuf.length()>0) {
                usbCmdBuf.trim();
                if      (usbCmdBuf.equalsIgnoreCase("REG"))   {
                    ts(); Serial.println("[USB] REG: Hintergrundsuche reset + waitNetReg()...");
                    regState = REG_NOT_STARTED;
                    netRegistered = false;
                    waitNetReg();
                    regState = netRegistered ? REG_REGISTERED : REG_NOT_STARTED;
                }
                else if (usbCmdBuf.equalsIgnoreCase("DIAG"))  bootDiag();
                else if (usbCmdBuf.equalsIgnoreCase("SCAN"))  sendAT("AT+COPS=?","OK",180000);
                else if (usbCmdBuf.equalsIgnoreCase("RESTART")) hangRestart("manueller RESTART via USB");
                else if (usbCmdBuf.equalsIgnoreCase("DAILY"))   dailyRestart("manuell via USB (Test)");
                else if (usbCmdBuf.equalsIgnoreCase("LOG"))     logPrintAll();
                else if (usbCmdBuf.equalsIgnoreCase("LOGCLR"))  logClear();
                else if (usbCmdBuf.length()>0)                sendAT(usbCmdBuf.c_str(),"OK",10000);
                usbCmdBuf="";
            }
        } else {
            usbCmdBuf+=c;
            if (usbCmdBuf.length()>200) usbCmdBuf="";
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 46: Modem-Lebenszeichen pruefen (im Leerlauf)
// ══════════════════════════════════════════════════════════════════════════════
static void checkModemAlive() {
    static uint32_t lastHealthCheck = 0;
    uint32_t now = millis();

    // Leerlauf-Ping: nur wenn laenger nichts vom Modem kam
    if (now - lastModemAliveMs > MODEM_HEALTH_INTERVAL_MS &&
        now - lastHealthCheck  > MODEM_HEALTH_INTERVAL_MS) {
        lastHealthCheck = now;
        if (!sendAT("AT","OK",T_SHORT)) {
            logEvent("WARN", "Modem antwortet nicht auf AT (stumm seit %lus)",
                     (unsigned long)((millis() - lastModemAliveMs)/1000UL));
            ts(); Serial.print("[HEALTH] Modem antwortet nicht (stumm seit ");
            Serial.print((millis() - lastModemAliveMs)/1000);
            Serial.print("s, Neustart bei ");
            Serial.print(HANG_RESTART_S); Serial.println("s)");
        }
    }

    if (millis() - lastModemAliveMs > HANG_RESTART_MS) {
        hangRestart("Modem seit 5 Minuten ohne Antwort");
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 47: Taeglicher Neustart ATOM + SIM7028 mit 1 Min Pause
// ══════════════════════════════════════════════════════════════════════════════
static void dailyRestart(const char *reason) {
    dailyRestartCount++;
    Serial.println();
    ts(); Serial.print("[DAILY] Neustart #"); Serial.print(dailyRestartCount);
    Serial.print(" ("); Serial.print(reason); Serial.println(")");
    logEvent("INFO", "Tages-Neustart (%s) | Bilanz: HTTP ok=%lu fehl=%lu, SAPBR-ERROR=%lu",
             reason, (unsigned long)statHttpOk, (unsigned long)statHttpFail,
             (unsigned long)statSapbrErr);
    rtcEpochAfterSleep = timeValid ? (nowEpoch() + (uint32_t)DAILY_RESTART_OFF_S) : 0;

    // 1. Offene HTTP-Session beenden
    if (httpInitDone) { sendAT("AT+HTTPTERM","OK",T_SHORT); httpInitDone=false; }

    // 2. Modem sauber abschalten
    ts(); Serial.println("[DAILY] SIM7028 abschalten (AT+CPOWD=1)...");
    if (!sendAT("AT+CPOWD=1","NORMAL POWER DOWN",T_MEDIUM)) {
        ts(); Serial.println("[DAILY] kein NORMAL POWER DOWN - Fallback AT+CFUN=0");
        sendAT("AT+CFUN=0","OK",T_CFUN);
    }

    // 3. Power-Key-Pin im Sleep fest auf LOW halten
    pinMode(SIM_PWR_PIN, OUTPUT);
    digitalWrite(SIM_PWR_PIN, LOW);
    gpio_hold_en((gpio_num_t)SIM_PWR_PIN);
    gpio_deep_sleep_hold_en();

    // 4. ATOM fuer 60s ausschalten (Deep Sleep), danach Kaltstart
    ts(); Serial.print("[DAILY] ATOM geht fuer ");
    Serial.print((uint32_t)DAILY_RESTART_OFF_S);
    Serial.println("s aus (Deep Sleep) - danach Neustart von ATOM + Modem.");
    Serial.flush(); extSerial.flush();
    led.setPixelColor(0, 0); led.show();
    esp_sleep_enable_timer_wakeup(DAILY_RESTART_OFF_S * 1000000ULL);
    esp_deep_sleep_start();
}

static void checkDailyRestart() {
    if (millis() < DAILY_RESTART_MS) return;

    bool overdue = millis() > (DAILY_RESTART_MS + DAILY_RESTART_MAX_DELAY_MS);
    bool idle    = (millis() - lastExtCmdMs > DAILY_RESTART_IDLE_MS) && !httpInitDone;

    if (idle)         dailyRestart("24h Betriebszeit erreicht");
    else if (overdue) dailyRestart("24h + 2h - µC nie still, erzwungen");
    // sonst: µC gerade aktiv -> beim naechsten loop()-Durchlauf erneut pruefen
}

// ══════════════════════════════════════════════════════════════════════════════
// FIX 48: Netzzeit vom Modem holen (AT+CCLK?, NITZ)
// ══════════════════════════════════════════════════════════════════════════════
static void syncTimeFromModem() {
    if (!sendAT("AT+CCLK?","OK",T_SHORT)) return;
    const char *p = strstr(rxBuf, "+CCLK:");
    int yy, mo, dd, hh, mi, ss;
    if (!p || sscanf(p, "+CCLK: \"%d/%d/%d,%d:%d:%d", &yy,&mo,&dd,&hh,&mi,&ss) != 6) return;
    if (yy < 24 || yy > 99 || mo < 1 || mo > 12 || dd < 1 || dd > 31) {
        ts(); Serial.println("[TIME] Modem hat (noch) keine gueltige Netzzeit");
        return;
    }
    uint32_t ep = (uint32_t)daysFromCivil(2000+yy, mo, dd) * 86400UL
                + hh*3600UL + mi*60UL + ss;
    bool first = !timeValid;
    timeEpochBase = ep; timeBaseMs = millis(); timeSyncMs = millis(); timeValid = true;
    ts(); Serial.print("[TIME] Netzzeit gesetzt: "); Serial.println(logSnippet(p, 40));
    if (first) logEvent("INFO", "Netzzeit synchronisiert");
}

static void maybeSyncTime() {
    static uint32_t lastTry = 0;
    if (regState != REG_REGISTERED) return;
    if (timeValid && millis() - timeSyncMs < TIME_RESYNC_MS) return;
    if (lastTry != 0 && millis() - lastTry < 600000UL) return;   // max. alle 10 Min versuchen
    lastTry = millis();
    syncTimeFromModem();
}

// ══════════════════════════════════════════════════════════════════════════════
// Setup / Loop
// ══════════════════════════════════════════════════════════════════════════════
static const char* resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_UNKNOWN:   return "unbekannt (ESP_RST_UNKNOWN)";
        case ESP_RST_POWERON:   return "Power-On (Stromzyklus)";
        case ESP_RST_EXT:       return "externer Reset-Pin";
        case ESP_RST_SW:        return "Software-Reset (z.B. ESP.restart() / Haenger-Neustart)";
        case ESP_RST_PANIC:     return "Software-PANIC (Crash/Exception oder Task-WDT)";
        case ESP_RST_INT_WDT:   return "interner Watchdog (Task/Interrupt)";
        case ESP_RST_TASK_WDT:  return "Task-Watchdog-Timeout (5-Min-Haenger)";
        case ESP_RST_WDT:       return "sonstiger Watchdog";
        case ESP_RST_DEEPSLEEP: return "Aufwachen aus Deep Sleep";
        case ESP_RST_BROWNOUT:  return "⚠ BROWNOUT (Spannungseinbruch!) ⚠";
        case ESP_RST_SDIO:      return "SDIO-Reset";
#ifdef ESP_RST_USB
        case ESP_RST_USB:       return "USB-Reset";
#endif
#ifdef ESP_RST_JTAG
        case ESP_RST_JTAG:      return "JTAG-Reset";
#endif
#ifdef ESP_RST_EFUSE
        case ESP_RST_EFUSE:     return "eFuse-Fehler";
#endif
#ifdef ESP_RST_PWR_GLITCH
        case ESP_RST_PWR_GLITCH: return "⚠ Power-Glitch (kurzer Spannungsstoerimpuls) ⚠";
#endif
#ifdef ESP_RST_CPU_LOCKUP
        case ESP_RST_CPU_LOCKUP: return "CPU-Lockup";
#endif
        default:                return "unbekannt (nicht im Code abgedeckter Wert)";
    }
}

void setup() {
    wdtInit();   // FIX 46: zuerst – deckt auch einen Haenger im Setup ab

    gpio_hold_dis((gpio_num_t)SIM_PWR_PIN);   // FIX 47: Hold aus dem Sleep loesen
    gpio_deep_sleep_hold_dis();

    Serial.begin(115200);
    delay(500);

    simSerial.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
    extSerial.setRxBufferSize(1024);   // FIX 48: Puffer fuer Flash-Schreibpausen
    extSerial.begin(EXT_BAUD,  SERIAL_8N1, EXT_RX_PIN, EXT_TX_PIN);

    led.begin(); led.setBrightness(80); ledBlue();
    extCmdBuf.reserve(512);
    usbCmdBuf.reserve(256);

    Serial.println("\n╔══════════════════════════════════════════════╗");
    Serial.printf( "║  SIM800→SIM7028 Bridge %s (%s)   ║\n", FW_VERSION, FW_DATE);
    Serial.println("║  SIM: RX=G6 TX=G5  |  EXT: TX=G38 RX=G39   ║");
    Serial.println("║  PDP-Kontext 0 | 1NCE No-Harm Backoff        ║");
    Serial.println("║  Kein Sleep | Haenger-Neustart nach 5 Min    ║");
    Serial.println("║  Taeglicher Neustart ATOM+SIM (1 Min Pause)  ║");
    Serial.println("║  Fehler-Log im Flash (LOG / LOGCLR)          ║");
    Serial.println("║  USB: REG|DIAG|SCAN|RESTART|DAILY|LOG|AT+... ║");
    Serial.println("╚══════════════════════════════════════════════╝");

    esp_reset_reason_t rr = esp_reset_reason();
    ts(); Serial.print("[BOOT] Reset-Grund: ");
    Serial.print(resetReasonName(rr));
    Serial.print("  (Rohwert: "); Serial.print((int)rr); Serial.println(")");
    if (rr == ESP_RST_DEEPSLEEP) {
        ts(); Serial.print("[BOOT] -> Taeglicher Neustart #");
        Serial.print(dailyRestartCount); Serial.println(" abgeschlossen (nach 1 Min Pause)");
    }
    // FIX 48: Flash-Log starten, Zeit ueber Deep Sleep retten, Boot loggen
    logInit();
    if (rr == ESP_RST_DEEPSLEEP && rtcEpochAfterSleep != 0) {
        timeEpochBase = rtcEpochAfterSleep; timeBaseMs = 0;
        timeSyncMs = millis(); timeValid = true;
    }
    rtcEpochAfterSleep = 0;
    {
        bool bad = (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
                    rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT ||
                    rr == ESP_RST_BROWNOUT);
        logEvent(bad ? "ERR" : "BOOT", "Start %s, Reset-Grund: %s",
                 FW_VERSION, resetReasonName(rr));
    }
    ts(); Serial.print("[BOOT] Freier Heap: ");
    Serial.print(ESP.getFreeHeap()); Serial.println(" Bytes");

    lastModemAliveMs = millis();
    ensureModemOn();
    setApn();
    sendAT("AT+CEREG=1","OK",T_SHORT);

    startRegistrationIfNeeded();

    {
        uint32_t tf=millis(); int flushed=0;
        while (millis()-tf < 500) {
            while (simSerial.available()) { simSerial.read(); flushed++; }
            wdtYield();
        }
        if (flushed>0) { ts(); Serial.print("[BOOT] Modem-Puffer geleert: ");
                         Serial.print(flushed); Serial.println(" Bytes verworfen"); }
    }
    lastDataLen=0; bodyLen=0; rawHttpReadLen=0;

    ledOff(); ledSet(0,80,0,500);
    ts(); Serial.println("[OK] Bridge " FW_VERSION " bereit\n");
}

void loop() {
    esp_task_wdt_reset();   // FIX 46: Lebenszeichen der Hauptschleife

    while (simSerial.available()) {
        static char urcBuf[256];
        static uint16_t urcIdx = 0;
        char c = (char)simSerial.read();
        lastModemAliveMs = millis();   // FIX 46: URC = Modem lebt
        if (c == '\n') {
            if (urcIdx > 0) {
                urcBuf[urcIdx] = '\0';
                if (urcIdx > 1 || urcBuf[0] != '\r') {
                    ts(); Serial.print("[←SIM URC] ");
                    printEscaped(urcBuf, urcIdx);
                    Serial.println();
                }
                urcIdx = 0;
            }
        } else {
            if (urcIdx < sizeof(urcBuf)-1) urcBuf[urcIdx++] = c;
        }
    }

    readExtSerial();
    readUsbSerial();
    pollRegistrationBackground();
    checkModemAlive();      // FIX 46
    checkDailyRestart();    // FIX 47
    maybeSyncTime();        // FIX 48

    static uint32_t lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        lastHeartbeat = millis();
        const char *stateStr =
            (regState==REG_NOT_STARTED) ? "NOT_STARTED" :
            (regState==REG_SEARCHING)   ? "SEARCHING"   :
            (regState==REG_REGISTERED)  ? "REGISTERED"  :
            (regState==REG_GAVE_UP)     ? "GAVE_UP"     : "?";
        ts(); Serial.print("[HEARTBEAT] uptime=");
        Serial.print(millis()/1000); Serial.print("s regState=");
        Serial.print(stateStr);
        if (regState == REG_SEARCHING) {
            Serial.print(" attempt=#"); Serial.print(regAttempts+1);
            Serial.print("/"); Serial.print(REG_MAX_ATTEMPTS);
            uint32_t elapsed = millis() - regLastCopsTrigger;
            uint32_t remain = (elapsed < regBackoffMs) ? (regBackoffMs - elapsed) : 0;
            Serial.print(" naechsterRetrigger=in "); Serial.print(remain/1000); Serial.print("s");
        }
        Serial.print(" modemStumm="); Serial.print((millis()-lastModemAliveMs)/1000); Serial.print("s");
        Serial.print(" dailyRestartIn=");
        Serial.print(millis() < DAILY_RESTART_MS ? (DAILY_RESTART_MS - millis())/60000UL : 0);
        Serial.print("min");
        Serial.print(" heap="); Serial.print(ESP.getFreeHeap()); Serial.println("B");
    }

    yield();
}
