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

#define BUS Serial1
static const uint8_t TX_PIN = 16;
static const uint8_t RX_PIN = 17;
static const uint32_t BUS_BAUD = 1000000;

static const uint8_t BROADCAST_ID = 0xFE;
static const uint8_t INST_SYNC_READ = 0x82;
static const int MAX_PACKET = 260;   // 4-byte header + LEN (max 255) + 1
static const int MAX_REPLIES = 32;

static const uint32_t HOST_STALL_US = 3000;     // a half-received USB packet older than this is dropped
static const uint32_t ECHO_WINDOW_US = 150;     // our echo has arrived by then (line released at t=0)
static const uint32_t REPLY_BASE_US = 1500;     // first reply may take this long
static const uint32_t REPLY_EACH_US = 600;      // plus per expected reply, on top of its wire time
static const uint32_t BYTE_US = 10;             // 1 Mbps, 8N1

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

// ---- host -> bridge: assemble one instruction packet ----------------------------------------
static uint8_t inPkt[MAX_PACKET];
static int inLen = 0;
static uint32_t inLastByte = 0;

// Feed one USB byte. Returns true when inPkt holds a complete packet with a valid checksum.
static bool feedHostByte(uint8_t b)
{
    if (inLen > 0 && micros() - inLastByte > HOST_STALL_US) inLen = 0;   // stale fragment
    inLastByte = micros();

    if (inLen < 2)
    {
        if (b == 0xFF) inPkt[inLen++] = b;
        else inLen = 0;
        return false;
    }
    if (inLen == 2 && b == 0xFF) return false;   // FF FF FF: stay aligned on the last two
    inPkt[inLen++] = b;
    if (inLen < 4) return false;

    int len = inPkt[3];
    if (len < 2) { inLen = 0; return false; }
    if (inLen < 4 + len) return false;

    bool ok = inPkt[3 + len] == checksum(&inPkt[2], len + 1);
    int total = inLen;
    inLen = 0;
    return ok && total == 4 + len;
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
static uint32_t errorCount = 0;
static uint32_t lastErrorMs = 0, lastActivityMs = 0;

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
    Serial.write(m, sizeof(m));
    Serial.flush();
    errorCount++;
    lastErrorMs = millis();
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
        return;
    }
    static uint8_t out[MAX_REPLIES * MAX_PACKET];
    int outLen = 0;
    for (int i = 0; i < e.count; i++)
    {
        memcpy(&out[outLen], replies[i], replyLens[i]);
        outLen += replyLens[i];
    }
    Serial.write(out, outLen);   // the whole answer at once: nothing trickles in later
    Serial.flush();
}

// ---- Arduino ---------------------------------------------------------------------------------
void setup()
{
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
    while (Serial.available())
    {
        if (feedHostByte((uint8_t)Serial.read())) transact(inPkt, 4 + inPkt[3]);
    }

    // LED: fast blink for 1 s after an error, short flicker on traffic, otherwise off.
    uint32_t now = millis();
    bool led;
    if (errorCount && now - lastErrorMs < 1000) led = (now / 60) % 2;
    else led = now - lastActivityMs < 30;
    digitalWrite(LED_BUILTIN, led);
}
