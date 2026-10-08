// Jero servo bus bridge for the Raspberry Pi Pico (RP2040).
//
// The Pico replaces the Waveshare Bus Servo Adapter: it shows up on the Pi Zero 2 W as
// /dev/ttyACM0 and carries Feetech STS packets between USB and the single-wire servo bus at
// 1 Mbps, so upstream Open_Duck_Mini_Runtime (rustypot) and pypot work unchanged.
//
// It works per transaction, not per byte, because rustypot aborts on any stray or late byte
// ("assertion failed: self.is_input_buffer_empty"):
//   1. read one complete, checksummed instruction packet from USB;
//   2. work out the replies it must produce (sync write / broadcast: none; sync read: one per
//      listed ID; anything addressed to one ID: one);
//   3. send it on the bus, release the line, skip our own echo;
//   4. collect replies by parsing whole packets (header, expected ID, length, checksum) until
//      all have arrived or a deadline passes;
//   5. answer USB in ONE write: every reply in the order requested, or, if any reply is missing
//      or corrupt, ONE short error marker and nothing else.
// The error marker is a well-formed status packet from an ID that wasn't asked, so rustypot and
// pypot reject it immediately (ParsingError) instead of waiting 1 s, and nothing is left in the
// host's buffer. It never carries servo data.
//
// Wiring: GP16 (UART0 TX) straight to servo DATA; GP17 (UART0 RX) to DATA through ~5 kOhm;
// Pico GND to the servo supply GND. Servo power never goes through the Pico.

#include <Arduino.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>

#define BUS Serial1
static const uint8_t TX_PIN = 16;
static const uint8_t RX_PIN = 17;
static const uint32_t BUS_BAUD = 1000000;

static const uint8_t BROADCAST_ID = 0xFE;
static const uint8_t INST_SYNC_READ = 0x82;
static const uint8_t INST_BRIDGE_STATS = 0xF0;   // broadcast only; answered by the bridge, never sent to servos
static const int MAX_PACKET = 260;   // 4-byte header + LEN (max 255) + 1
static const int MAX_REPLIES = 32;

static const uint32_t HOST_STALL_US = 3000;     // a half-received USB packet older than this is dropped
static const uint32_t ECHO_WINDOW_US = 150;     // our echo has arrived by then (line released at t=0)
static const uint32_t REPLY_BASE_US = 1500;     // first reply may take this long
static const uint32_t REPLY_EACH_US = 600;      // plus per expected reply, on top of its wire time
static const uint32_t BYTE_US = 10;             // 1 Mbps, 8N1
static const uint32_t WATCHDOG_MS = 500;        // loop stuck this long -> the chip reboots itself
static const uint32_t USB_DRAIN_US = 5000;      // wait this long for a reply to leave the USB buffer

// ---- bus line --------------------------------------------------------------------------------
static bool released = false;

static void takeBus()
{
    if (released)
    {
        gpio_set_function(TX_PIN, GPIO_FUNC_UART);
        released = false;
    }
}

static void releaseBus()
{
    if (!released)
    {
        gpio_set_dir(TX_PIN, GPIO_IN);
        gpio_pull_up(TX_PIN);
        gpio_set_function(TX_PIN, GPIO_FUNC_SIO);
        released = true;
    }
}

static uint8_t checksum(const uint8_t *p, int n)   // over ID..last param
{
    uint8_t s = 0;
    for (int i = 0; i < n; i++) s += p[i];
    return (uint8_t)~s;
}

// ---- counters (read with tools/bridge_stats.py) ----------------------------------------------
struct Stats
{
    uint32_t requests;      // complete, valid instruction packets from the host
    uint32_t badChecksum;   // complete packets from the host with a wrong checksum (answered with a marker)
    uint32_t stalled;       // host packets that stopped half way (answered with a marker)
    uint32_t junkBytes;     // host bytes outside any packet
    uint32_t replyErrors;   // bus transactions with a missing or corrupt servo reply (marker sent)
    uint32_t okBatches;     // bus transactions answered in full
    uint32_t usbShort;      // USB writes that didn't take every byte
    uint32_t watchdogResets; // reboots by our watchdog since power-on (kept in a watchdog scratch register)
    uint32_t usbSlow;       // replies still in the USB send buffer after USB_DRAIN_US
};
static Stats stats = {};

// ---- host -> bridge: assemble one instruction packet ----------------------------------------
static uint8_t inPkt[MAX_PACKET];
static int inLen = 0;
static uint32_t inLastByte = 0;

enum HostResult { HOST_NONE, HOST_PACKET, HOST_BAD };

// Feed one USB byte. HOST_PACKET when inPkt holds a complete packet with a valid checksum.
static HostResult feedHostByte(uint8_t b)
{
    inLastByte = micros();

    if (inLen < 2)
    {
        if (b == 0xFF) inPkt[inLen++] = b;
        else { inLen = 0; stats.junkBytes++; }
        return HOST_NONE;
    }
    if (inLen == 2 && b == 0xFF) return HOST_NONE;   // FF FF FF: stay aligned on the last two
    inPkt[inLen++] = b;
    if (inLen < 4) return HOST_NONE;

    int len = inPkt[3];
    if (len < 2) { inLen = 0; return HOST_BAD; }
    if (inLen < 4 + len) return HOST_NONE;

    inLen = 0;
    return inPkt[3 + len] == checksum(&inPkt[2], len + 1) ? HOST_PACKET : HOST_BAD;
}

// ---- one bus transaction ---------------------------------------------------------------------
struct Expect
{
    int count = 0;
    uint8_t ids[MAX_REPLIES];
    int replyLen = -1;   // full reply length if known (sync read), -1 = any
};

static Expect expectedReplies(const uint8_t *pkt)
{
    Expect e;
    uint8_t id = pkt[2], len = pkt[3], inst = pkt[4];
    if (id == BROADCAST_ID)
    {
        if (inst == INST_SYNC_READ && len >= 4)
        {
            int n = len - 4;   // LEN = INST + addr + datalen + ids... + CHK
            if (n > MAX_REPLIES) n = MAX_REPLIES;
            for (int i = 0; i < n; i++) e.ids[i] = pkt[7 + i];
            e.count = n;
            e.replyLen = 6 + pkt[6];   // FF FF ID LEN ERR data[datalen] CHK
        }
        return e;   // other broadcasts (sync write, ...) get no reply
    }
    e.ids[0] = id;
    e.count = 1;
    return e;
}

static uint8_t replies[MAX_REPLIES][MAX_PACKET];
static int replyLens[MAX_REPLIES];
static uint8_t rx[MAX_REPLIES * 40 + MAX_PACKET];
static uint32_t lastReplyErrorMs = 0, lastHostErrorMs = 0, lastActivityMs = 0;
static bool anyReplyError = false, anyHostError = false;

static int usbTxCapacity = 0;   // free space of an empty USB send buffer (largest seen)

// Write a reply and make sure it has actually left the USB send buffer before the next request
// is handled: a reply that sits in the buffer and goes out with the next one is exactly the
// "late reply" the Pi can't tell apart from a fresh one.
static void usbWrite(const uint8_t *buf, size_t n)
{
    if (Serial.write(buf, n) < n) stats.usbShort++;
    Serial.flush();
    uint32_t t0 = micros();
    while (Serial.availableForWrite() < usbTxCapacity)
    {
        if (micros() - t0 > USB_DRAIN_US) { stats.usbSlow++; break; }
        Serial.flush();
        delayMicroseconds(20);
    }
}

static void sendErrorMarker(const Expect &e)
{
    // A complete status packet from an ID nobody asked for: rejected at once, nothing left over.
    uint8_t mid = 0;
    for (bool clash = true; clash && mid < 0xFD;)
    {
        clash = false;
        for (int i = 0; i < e.count; i++)
            if (e.ids[i] == mid) { clash = true; mid++; break; }
    }
    uint8_t m[6] = {0xFF, 0xFF, mid, 0x02, 0x00, 0};
    m[5] = checksum(&m[2], 3);
    usbWrite(m, sizeof(m));
}

// The host sent something we can't act on (bad checksum, or it stopped half way). It is very
// likely waiting for an answer: fail it now instead of letting it time out after 1 s.
static void hostError()
{
    Expect none;
    sendErrorMarker(none);
    anyHostError = true;
    lastHostErrorMs = millis();
}

static void sendStats()
{
    const uint32_t v[] = {stats.requests, stats.badChecksum, stats.stalled, stats.junkBytes,
                          stats.replyErrors, stats.okBatches, stats.usbShort, stats.watchdogResets,
                          stats.usbSlow};
    const int n = sizeof(v);
    uint8_t m[6 + n];
    m[0] = 0xFF; m[1] = 0xFF; m[2] = BROADCAST_ID; m[3] = n + 2; m[4] = 0x00;
    memcpy(&m[5], v, n);   // little-endian uint32s, in Stats order
    m[5 + n] = checksum(&m[2], n + 3);
    usbWrite(m, sizeof(m));
}

static void transact(const uint8_t *pkt, int n)
{
    Expect e = expectedReplies(pkt);
    lastActivityMs = millis();

    while (BUS.available()) BUS.read();   // nothing from before this transaction is forwarded
    takeBus();
    BUS.write(pkt, n);
    BUS.flush();                          // returns once the last stop bit has left the pin
    releaseBus();                         // free the line before the first servo can answer
    uint32_t t0 = micros();
    if (e.count == 0)
    {
        // No reply expected (sync write): wait out our echo so it can't spill into the next
        // transaction, then drop it.
        while (micros() - t0 < ECHO_WINDOW_US) {}
        while (BUS.available()) BUS.read();
        return;
    }

    // Skip our own echo: the first n bytes heard within the echo window.
    int echo = 0;
    while (echo < n && micros() - t0 < ECHO_WINDOW_US)
    {
        if (BUS.available()) { BUS.read(); echo++; }
    }

    int wire = e.replyLen > 0 ? e.replyLen * BYTE_US : 8 * BYTE_US;
    uint32_t deadline = REPLY_BASE_US + (uint32_t)e.count * (wire + REPLY_EACH_US);
    bool got[MAX_REPLIES] = {false};
    int have = 0, rxLen = 0, pos = 0;

    while (have < e.count && micros() - t0 < deadline)
    {
        while (BUS.available() && rxLen < (int)sizeof(rx)) rx[rxLen++] = BUS.read();

        // parse as many whole packets as the buffer holds
        while (rxLen - pos >= 6)
        {
            if (rx[pos] != 0xFF || rx[pos + 1] != 0xFF) { pos++; continue; }
            if (rx[pos + 2] == 0xFF) { pos++; continue; }   // FF FF FF: realign
            int len = rx[pos + 3];
            int total = 4 + len;
            if (len < 2 || total > MAX_PACKET) { pos++; continue; }
            if (rxLen - pos < total) break;                 // wait for the rest
            const uint8_t *p = &rx[pos];
            if (total == n && memcmp(p, pkt, n) == 0) { pos += total; continue; }   // a late echo of our own packet
            int slot = -1;
            for (int i = 0; i < e.count; i++)
                if (!got[i] && e.ids[i] == p[2]) { slot = i; break; }
            bool lenOk = e.replyLen < 0 || total == e.replyLen;
            if (slot >= 0 && lenOk && p[total - 1] == checksum(&p[2], total - 3))
            {
                memcpy(replies[slot], p, total);
                replyLens[slot] = total;
                got[slot] = true;
                have++;
                pos += total;
            }
            else
            {
                pos++;   // corrupt or unexpected: resync on the next header
            }
        }
    }

    if (have < e.count)
    {
        sendErrorMarker(e);
        stats.replyErrors++;
        anyReplyError = true;
        lastReplyErrorMs = millis();
        return;
    }
    static uint8_t out[MAX_REPLIES * MAX_PACKET];
    int outLen = 0;
    for (int i = 0; i < e.count; i++)
    {
        memcpy(&out[outLen], replies[i], replyLens[i]);
        outLen += replyLens[i];
    }
    usbWrite(out, outLen);   // the whole answer at once: nothing trickles in later
    stats.okBatches++;
}

// ---- Arduino ---------------------------------------------------------------------------------
void setup()
{
    // scratch[0] survives a watchdog reboot but not a power cycle: count our own watchdog resets
    uint32_t resets = watchdog_enable_caused_reboot() ? watchdog_hw->scratch[0] + 1 : 0;
    watchdog_hw->scratch[0] = resets;
    stats.watchdogResets = resets;
    watchdog_enable(WATCHDOG_MS, true);   // true: paused while a debugger halts the core

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(BUS_BAUD);   // USB CDC: the host's baud setting is ignored
    BUS.setTX(TX_PIN);
    BUS.setRX(RX_PIN);
    BUS.setFIFOSize(1024);
    BUS.begin(BUS_BAUD, SERIAL_8N1);
    releaseBus();
}

void loop()
{
    watchdog_update();   // a transaction takes at most ~15 ms; anything stuck for 500 ms reboots
    int free = Serial.availableForWrite();
    if (free > usbTxCapacity) usbTxCapacity = free;   // learnt while idle, before any reply
    while (Serial.available())
    {
        HostResult r = feedHostByte((uint8_t)Serial.read());
        if (r == HOST_PACKET)
        {
            stats.requests++;
            if (inPkt[2] == BROADCAST_ID && inPkt[4] == INST_BRIDGE_STATS) sendStats();
            else transact(inPkt, 4 + inPkt[3]);
        }
        else if (r == HOST_BAD)
        {
            stats.badChecksum++;
            hostError();
        }
    }
    if (inLen > 0 && micros() - inLastByte > HOST_STALL_US)
    {
        inLen = 0;   // a packet that stopped half way
        stats.stalled++;
        hostError();
    }

    // LED: slow blink for 1.2 s after a bad/stalled host packet, fast blink for 1 s after a
    // servo reply error, short flicker on traffic, otherwise off.
    uint32_t now = millis();
    bool led;
    if (anyHostError && now - lastHostErrorMs < 1200) led = ((now - lastHostErrorMs) / 300) % 2 == 0;
    else if (anyReplyError && now - lastReplyErrorMs < 1000) led = (now / 60) % 2;
    else led = now - lastActivityMs < 30;
    digitalWrite(LED_BUILTIN, led);
}
