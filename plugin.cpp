// MK7 Bot v3 - CTRPluginFramework (3GX) - UNTESTED on hardware.
// Finds its own path by trailing the lead CPU kart on lap 1, then drives the learned line,
// self-tunes, brakes for corners, recovers from being stuck, and uses items by situation.
// SD files: /MK7Bot/line_<course>.bin  params_<course>.bin  policy.bin
#include <3ds.h>
#include <CTRPluginFramework.hpp>
#include <cmath>
#include <string>
#include <vector>
using namespace CTRPluginFramework;

// ====================== CONFIG: ALL OF THIS NEEDS REAL VALUES ======================
static const u32  ADDR_KART_TABLE = 0x0;    // base of the 8-kart list
static const bool kPtrTable = false;        // true if the list holds pointers to kart objects
static const u32  kStride = 0x0;            // kart struct size when !kPtrTable
static const int  kKarts = 8;
static const int  kPlayer = 0;              // index of the human kart
static const u32  OFF_X = 0x0, OFF_Z = 0x0, OFF_YAW = 0x0, OFF_SPEED = 0x0, OFF_RANK = 0x0, OFF_ITEM = 0x0;
static const u32  ADDR_IN_RACE = 0x0;       // u32, nonzero while racing (after "GO")
static const u32  ADDR_COURSE_ID = 0x0;     // u32, current course id (0 is fine if unknown)

static const float kMaxSpeed = 100.f;       // speed units at full speed
static const float kSpacing = 30.f;         // waypoint spacing in world units (tune to track scale)
static const int   kLook = 3;               // shared with Python env
static const float kPolicyRadius = 40.f;    // shared with Python env
#define HAVE_CIRCLEPAD_INJECT 0             // set 1 if your CTRPF has analog circle-pad injection
// ====================================================================================
static const int kObs = 8, kH = 32, kAct = 3;

struct Vec2 { float x, z; };
struct Params { float gain = 1.6f, radius = 40.f, look = 3.f, brakeAngle = 0.35f, brakeSpeed = 0.7f; };
struct Kart { float x = 0, z = 0, yaw = 0, speed = 0; u32 rank = 0, item = 0; };
enum Mode { M_IDLE, M_FOLLOW, M_RACE };

static Mode g_mode = M_IDLE;
static bool g_botOn = false, g_tune = true, g_usePolicy = false, g_rec = false;
static u32  g_cid = 0;
static std::vector<Vec2>  g_line, g_hit, g_trail;
static std::vector<float> g_curv;
static float g_trailLen = 0;
static size_t g_target = 0;
static int g_leader = -1, g_visited = 0, g_lapFrames = 0, g_hits = 0, g_stuckEvents = 0;
static Params g_cur, g_best;
static float g_bestCost = 1e9f;
static const char *kDir = "/MK7Bot/";

// ---------- math / rng ----------
static u32 g_rng = 2463534242u;
static float Rand01() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return (g_rng & 0xFFFFFF) / 16777216.f; }
static float Gauss() { return (Rand01() + Rand01() + Rand01() + Rand01() - 2.f) * 1.73f; }
static float Wrap(float a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }
static float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float Dist(float ax, float az, float bx, float bz) { return hypotf(ax - bx, az - bz); }

// ---------- karts ----------
static u32 KartBase(int i)
{
    if (kPtrTable) { u32 p = 0; return Process::Read32(ADDR_KART_TABLE + 4 * i, p) ? p : 0; }
    return ADDR_KART_TABLE + i * kStride;
}
static bool ReadKartN(int i, Kart &k)
{
    u32 b = KartBase(i);
    if (!b) return false;
    return Process::ReadFloat(b + OFF_X, k.x) && Process::ReadFloat(b + OFF_Z, k.z)
        && Process::ReadFloat(b + OFF_YAW, k.yaw) && Process::ReadFloat(b + OFF_SPEED, k.speed)
        && Process::Read32(b + OFF_RANK, k.rank) && Process::Read32(b + OFF_ITEM, k.item);
}

// ---------- files ----------
static std::string LineName() { return Utils::Format("line_%08X.bin", g_cid); }
static std::string ParamName() { return Utils::Format("params_%08X.bin", g_cid); }
static bool LoadFile(const std::string &name, std::vector<u8> &out)
{
    File f;
    if (File::Open(f, std::string(kDir) + name, File::READ) != 0) return false;
    out.resize(f.GetSize());
    bool ok = f.Read(out.data(), out.size()) == 0;
    f.Close();
    return ok;
}
static void SaveFile(const std::string &name, const void *d, size_t n)
{
    Directory::Create(kDir);
    File f;
    if (File::Open(f, std::string(kDir) + name, File::RWC | File::TRUNCATE) == 0) { f.Write(d, n); f.Close(); }
}

// ---------- line processing ----------
static void Resample(const std::vector<Vec2> &in, float spacing, std::vector<Vec2> &out)
{
    out.clear();
    if (in.size() < 2) return;
    std::vector<Vec2> p = in;
    p.push_back(in[0]);                                   // close the loop
    out.push_back(p[0]);
    float need = spacing;
    for (size_t i = 1; i < p.size(); i++) {
        Vec2 a = p[i - 1], b = p[i];
        float seg = Dist(a.x, a.z, b.x, b.z), pos = 0;
        while (seg - pos >= need) {
            pos += need;
            float t = pos / seg;
            out.push_back({a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t});
            need = spacing;
        }
        need -= (seg - pos);
    }
    if (out.size() > 2 && Dist(out.back().x, out.back().z, out[0].x, out[0].z) < spacing * 0.5f) out.pop_back();
}
static void Smooth(std::vector<Vec2> &p, int passes)
{
    size_t n = p.size();
    if (n < 3) return;
    for (int k = 0; k < passes; k++) {
        std::vector<Vec2> q = p;
        for (size_t i = 0; i < n; i++) {
            const Vec2 &a = p[(i + n - 1) % n], &b = p[i], &c = p[(i + 1) % n];
            q[i] = {0.25f * a.x + 0.5f * b.x + 0.25f * c.x, 0.25f * a.z + 0.5f * b.z + 0.25f * c.z};
        }
        p = q;
    }
}
static void ComputeCurv()
{
    size_t n = g_line.size();
    g_curv.assign(n, 0.f);
    for (size_t i = 0; i < n; i++) {
        const Vec2 &a = g_line[(i + n - 1) % n], &b = g_line[i], &c = g_line[(i + 1) % n];
        g_curv[i] = fabsf(Wrap(atan2f(c.x - b.x, c.z - b.z) - atan2f(b.x - a.x, b.z - a.z)));
    }
}
static void SaveLine()
{
    std::vector<float> v;
    for (auto &p : g_line) { v.push_back(p.x); v.push_back(p.z); }
    SaveFile(LineName(), v.data(), v.size() * 4);
}
static void RebuildFromRaw(std::vector<Vec2> raw)
{
    std::vector<Vec2> t;
    Resample(raw, kSpacing, t);
    Smooth(t, 3);
    Resample(t, kSpacing, g_line);
    g_hit = g_line;
    ComputeCurv();
}
static size_t Nearest(const std::vector<Vec2> &P, float x, float z)
{
    size_t b = 0; float bd = 1e18f;
    for (size_t i = 0; i < P.size(); i++) {
        float d = Dist(P[i].x, P[i].z, x, z);
        if (d < bd) { bd = d; b = i; }
    }
    return b;
}

// ---------- per-track load ----------
static void LoadTrack()
{
    g_line.clear(); g_hit.clear(); g_trail.clear(); g_curv.clear();
    g_trailLen = 0; g_bestCost = 1e9f; g_best = Params(); g_cur = g_best;
    std::vector<u8> b;
    if (LoadFile(LineName(), b) && b.size() >= 8 * 4) {
        std::vector<Vec2> raw;
        const float *f = (const float *)b.data();
        for (size_t i = 0; i + 1 < b.size() / 4; i += 2) raw.push_back({f[i], f[i + 1]});
        g_line = raw; g_hit = raw; ComputeCurv();
    }
    if (LoadFile(ParamName(), b) && b.size() == sizeof(Params)) { g_best = *(Params *)b.data(); g_cur = g_best; g_bestCost = 1e8f; }
}
static void StartMode() { g_mode = g_line.size() >= 8 ? M_RACE : M_FOLLOW; }

// ---------- neural policy ----------
static std::vector<float> g_net;
static bool LoadPolicy()
{
    std::vector<u8> b;
    if (!LoadFile("policy.bin", b)) return false;
    g_net.assign((float *)b.data(), (float *)b.data() + b.size() / 4);
    return g_net.size() == (size_t)(kObs * kH + kH + kH * kH + kH + kH * kAct + kAct);
}
static void Dense(const float *in, int ni, const float *&p, int no, float *out, bool act)
{
    const float *W = p, *B = p + ni * no;
    for (int o = 0; o < no; o++) {
        float s = B[o];
        for (int i = 0; i < ni; i++) s += W[o * ni + i] * in[i];
        out[o] = act ? tanhf(s) : s;
    }
    p = B + no;
}
static void Policy(const float *obs, float *act)
{
    float h1[kH], h2[kH];
    const float *p = g_net.data();
    Dense(obs, kObs, p, kH, h1, true);
    Dense(h1, kH, p, kH, h2, true);
    Dense(h2, kH, p, kAct, act, false);
}
static void BuildObs(const Kart &k, float *o)
{
    const Vec2 &t1 = g_line[g_target % g_line.size()], &t2 = g_line[(g_target + kLook) % g_line.size()];
    float e1 = Wrap(atan2f(t1.x - k.x, t1.z - k.z) - k.yaw);
    float e2 = Wrap(atan2f(t2.x - k.x, t2.z - k.z) - k.yaw);
    o[0] = sinf(e1); o[1] = cosf(e1); o[2] = sinf(e2); o[3] = cosf(e2);
    o[4] = Clamp(k.speed / kMaxSpeed, -1.f, 1.f);
    o[5] = fminf(Dist(t1.x, t1.z, k.x, k.z) / 200.f, 1.f);
    o[6] = k.rank / 8.f;
    o[7] = k.item ? 1.f : 0.f;
}

// ---------- controller output ----------
static void SendSteer(float s)
{
    s = Clamp(s, -1.f, 1.f);
#if HAVE_CIRCLEPAD_INJECT
    // Controller::InjectCirclePad((s16)(s * 156), 0);   // adjust to your CTRPF version
#else
    // D-pad steering with pulse-width modulation to fake analog
    static int ph = 0;
    ph = (ph + 1) & 3;
    float m = fabsf(s);
    if (m > 0.08f && m * 4.f > ph + 0.5f) Controller::InjectKey(s < 0 ? Key::DPadLeft : Key::DPadRight);
#endif
}

// ---------- items ----------
enum ItemKind { IK_UNKNOWN = 0, IK_BOOST, IK_FORWARD, IK_BACKWARD, IK_NOW };
static unsigned char g_itemKind[256];
static void SetupItems()
{
    // TODO: fill with real MK7 item ids, e.g.
    // g_itemKind[ID_MUSHROOM] = IK_BOOST;   g_itemKind[ID_RED_SHELL] = IK_FORWARD;
    // g_itemKind[ID_BANANA] = IK_BACKWARD;  g_itemKind[ID_STAR] = IK_NOW;
}
static int g_itemHeld = 0, g_itemCool = 0;
static u32 g_lastItem = 0;
static void UseItems(const Kart &me, const Kart *r, const bool *rok, float steerErr, bool braking)
{
    if (g_itemCool > 0) { g_itemCool--; return; }
    if (!me.item) { g_itemHeld = 0; g_lastItem = 0; return; }
    if (me.item != g_lastItem) { g_lastItem = me.item; g_itemHeld = 0; }
    g_itemHeld++;
    bool ahead = false, behind = false;
    for (int j = 0; j < kKarts; j++) {
        if (j == kPlayer || !rok[j]) continue;
        float rel = Wrap(atan2f(r[j].x - me.x, r[j].z - me.z) - me.yaw);
        float d = Dist(r[j].x, r[j].z, me.x, me.z);
        if (fabsf(rel) < 0.35f && d < kSpacing * 8) ahead = true;
        if (fabsf(rel) > 2.8f && d < kSpacing * 6) behind = true;
    }
    int kind = me.item < 256 ? (int)g_itemKind[me.item] : (int)IK_UNKNOWN;
    bool fire = false, back = false;
    switch (kind) {
        case IK_NOW:      fire = true; break;
        case IK_BOOST:    fire = fabsf(steerErr) < 0.15f && !braking; break;
        case IK_FORWARD:  fire = ahead; break;
        case IK_BACKWARD: fire = behind; back = true; break;
        default:          fire = ahead; break;
    }
    if (g_itemHeld > 300) fire = true;                       // never sit on an item forever
    if (fire) {
        Controller::InjectKey(back ? (Key::L | Key::DPadDown) : Key::L);   // TODO verify item button
        g_itemCool = 20;
    }
}

// ---------- self-tuning ----------
static Params Mutate(const Params &p)
{
    Params q = p;
    q.gain       = Clamp(p.gain * (1.f + 0.15f * Gauss()), 0.5f, 4.f);
    q.radius     = Clamp(p.radius * (1.f + 0.15f * Gauss()), 15.f, 120.f);
    q.look       = Clamp(p.look + 0.7f * Gauss(), 1.f, 8.f);
    q.brakeAngle = Clamp(p.brakeAngle * (1.f + 0.2f * Gauss()), 0.12f, 1.2f);
    q.brakeSpeed = Clamp(p.brakeSpeed + 0.06f * Gauss(), 0.3f, 0.95f);
    return q;
}
static void EndLap(bool valid, bool failed)
{
    float cost = g_lapFrames / 60.f + 1.5f * g_hits + 8.f * g_stuckEvents;
    if (failed) cost = 1e8f;
    if (valid || failed) {
        if (cost < g_bestCost) {
            g_bestCost = cost; g_best = g_cur;
            for (size_t i = 0; i < g_line.size(); i++) {      // pull line toward the path actually driven
                g_line[i].x += 0.25f * (g_hit[i].x - g_line[i].x);
                g_line[i].z += 0.25f * (g_hit[i].z - g_line[i].z);
            }
            ComputeCurv(); SaveLine(); SaveFile(ParamName(), &g_best, sizeof(Params));
        }
        g_cur = Mutate(g_best);
    }
    g_lapFrames = 0; g_visited = 0; g_hits = 0; g_stuckEvents = 0;
}

// ---------- race-start reset ----------
static void OnRaceStart(const Kart *r, const bool *rok, const Kart &me)
{
    g_lapFrames = 0; g_visited = 0; g_hits = 0; g_stuckEvents = 0; g_itemHeld = 0; g_itemCool = 0;
    if (g_mode == M_FOLLOW) {
        g_trail.clear(); g_trailLen = 0; g_target = 0; g_leader = -1;
        u32 best = 99;
        for (int j = 0; j < kKarts; j++)
            if (j != kPlayer && rok[j] && r[j].rank > 0 && r[j].rank < best) { best = r[j].rank; g_leader = j; }
    } else if (g_mode == M_RACE && !g_line.empty()) {
        g_target = Nearest(g_line, me.x, me.z);
    }
}

// ---------- the bot ----------
static int g_stuckFrames = 0, g_reverseFrames = 0;
static float g_prevSpeed = 0;
static void BotTick()
{
    static u32 wasRace = 0, lastCid = 0xFFFFFFFF;
    u32 cid = 0; Process::Read32(ADDR_COURSE_ID, cid);
    if (cid != lastCid) { lastCid = cid; g_cid = cid; LoadTrack(); if (g_botOn) StartMode(); }
    if (!g_botOn) { g_mode = M_IDLE; return; }

    u32 inRace = 0; Process::Read32(ADDR_IN_RACE, inRace);
    Kart me; if (!ReadKartN(kPlayer, me)) return;
    Kart r[kKarts]; bool rok[kKarts];
    for (int j = 0; j < kKarts; j++) rok[j] = (j != kPlayer) && ReadKartN(j, r[j]);
    if (inRace && !wasRace) OnRaceStart(r, rok, me);
    wasRace = inRace;
    if (!inRace) return;

    // learn the line by trailing the lead CPU
    if (g_mode == M_FOLLOW) {
        if (g_leader < 0 || !rok[g_leader]) return;
        const Kart &L = r[g_leader];
        if (g_trail.empty() || Dist(L.x, L.z, g_trail.back().x, g_trail.back().z) > kSpacing * 0.8f) {
            if (!g_trail.empty()) g_trailLen += Dist(L.x, L.z, g_trail.back().x, g_trail.back().z);
            g_trail.push_back({L.x, L.z});
            if (g_trailLen > 40 * kSpacing && g_trail.size() > 40 && Dist(L.x, L.z, g_trail[0].x, g_trail[0].z) < kSpacing * 2) {
                RebuildFromRaw(g_trail); SaveLine();
                g_mode = M_RACE; g_target = Nearest(g_line, me.x, me.z); g_lapFrames = 0; g_visited = 0;
                return;
            }
        }
    }

    // wall hits and stuck detection
    if (g_prevSpeed - me.speed > 0.2f * kMaxSpeed) g_hits++;
    g_prevSpeed = me.speed;
    g_stuckFrames = (me.speed < 0.03f * kMaxSpeed) ? g_stuckFrames + 1 : 0;
    if (g_stuckFrames > 120 && g_reverseFrames == 0) { g_reverseFrames = 50; g_stuckFrames = 0; g_stuckEvents++; }

    // pick target
    const std::vector<Vec2> &P = (g_mode == M_FOLLOW) ? g_trail : g_line;
    Vec2 tgt, aim;
    bool atHead = false;
    if (P.empty()) { if (g_mode == M_FOLLOW && g_leader >= 0 && rok[g_leader]) { tgt = aim = {r[g_leader].x, r[g_leader].z}; atHead = true; } else return; }
    else {
        if (g_target >= P.size()) g_target = P.size() - 1;
        float radius = (g_usePolicy || g_mode == M_FOLLOW) ? kPolicyRadius : g_cur.radius;
        tgt = P[g_target];
        if (Dist(tgt.x, tgt.z, me.x, me.z) < radius) {
            if (g_mode == M_RACE) {
                g_hit[g_target] = {me.x, me.z};
                g_visited++;
                g_target = (g_target + 1) % P.size();
                if (g_target == 0) { EndLap(g_visited >= (int)(0.9f * P.size()), false); }
            } else if (g_target + 1 < P.size()) g_target++;
        }
        tgt = P[g_target];
        size_t li = g_mode == M_RACE ? (g_target + (size_t)g_cur.look) % P.size() : g_target + 2;
        if (g_mode == M_FOLLOW && li >= P.size()) { atHead = true; aim = {r[g_leader].x, r[g_leader].z}; }
        else aim = P[li];
    }
    g_lapFrames++;
    if (g_mode == M_RACE && g_tune && g_lapFrames > 150 * 60) { EndLap(false, true); g_target = Nearest(g_line, me.x, me.z); }

    float err = Wrap(atan2f(aim.x - me.x, aim.z - me.z) - me.yaw);

    // neural policy
    if (g_usePolicy && g_mode == M_RACE && g_net.size()) {
        float o[kObs], a[kAct];
        BuildObs(me, o); Policy(o, a);
        if (a[1] > 0) Controller::InjectKey(Key::A);
        SendSteer(a[0]);
        if (a[2] > 0.5f && me.item) Controller::InjectKey(Key::L);
        return;
    }

    // reverse out of being stuck
    if (g_reverseFrames > 0) {
        g_reverseFrames--;
        Controller::InjectKey(Key::B);
        SendSteer(err > 0 ? -1.f : 1.f);
        return;
    }

    // throttle / brake
    bool accel = true, brake = false;
    if (g_mode == M_RACE && !g_curv.empty()) {
        float maxc = 0;
        for (int j = 0; j < 4; j++) maxc = fmaxf(maxc, g_curv[(g_target + j) % g_curv.size()]);
        if (maxc > g_cur.brakeAngle && me.speed > g_cur.brakeSpeed * kMaxSpeed) { accel = false; brake = maxc > 2 * g_cur.brakeAngle; }
    }
    if (g_mode == M_FOLLOW && atHead && g_leader >= 0 && rok[g_leader] &&
        Dist(r[g_leader].x, r[g_leader].z, me.x, me.z) < kSpacing * 3) accel = false;   // do not ram the leader
    if (accel) Controller::InjectKey(Key::A);
    if (brake) Controller::InjectKey(Key::B);

    SendSteer(err / 0.6f * g_cur.gain);   // flip the sign if the game's yaw is mirrored
    UseItems(me, r, rok, err, !accel);
}

// ---------- manual recording (optional fallback) ----------
static void RecordTick()
{
    static int f = 0;
    if (++f % 20) return;
    Kart k;
    if (ReadKartN(kPlayer, k)) g_trail.push_back({k.x, k.z});
}

// ---------- overlay ----------
static bool BotOverlay(const Screen &s)
{
    const std::vector<Vec2> &P = (g_mode == M_FOLLOW || g_line.size() < 2) ? g_trail : g_line;
    if (s.IsTop || P.size() < 2) return true;
    float mnx = 1e9, mxx = -1e9, mnz = 1e9, mxz = -1e9;
    for (auto &p : P) { mnx = fminf(mnx, p.x); mxx = fmaxf(mxx, p.x); mnz = fminf(mnz, p.z); mxz = fmaxf(mxz, p.z); }
    float sc = fminf(300.f / (mxx - mnx + 1), 200.f / (mxz - mnz + 1));
    auto Pt = [&](float x, float z, int &sx, int &sy) { sx = 10 + (int)((x - mnx) * sc); sy = 10 + (int)((z - mnz) * sc); };
    for (size_t i = 0; i < P.size(); i++) {
        int sx, sy; Pt(P[i].x, P[i].z, sx, sy);
        s.DrawRect(sx, sy, 2, 2, (i == g_target && g_mode != M_IDLE) ? Color::Yellow : Color::LimeGreen);
    }
    for (int j = 0; j < kKarts; j++) {
        Kart k;
        if (!ReadKartN(j, k)) continue;
        int sx, sy; Pt(k.x, k.z, sx, sy);
        if (j == kPlayer) s.DrawRect(sx - 2, sy - 2, 5, 5, Color::Red);
        else s.DrawRect(sx - 1, sy - 1, 3, 3, Color::Blue);
    }
    const char *m = g_mode == M_FOLLOW ? "LEARNING LINE" : (g_mode == M_RACE ? "DRIVING" : "IDLE");
    s.Draw(Utils::Format("%s  best cost %.1f  gain %.2f", m, g_bestCost, g_cur.gain), 10, 226);
    return true;
}

// ---------- menu ----------
static void BotFrame() { if (g_rec) RecordTick(); BotTick(); }
static void OnBot(MenuEntry *)
{
    g_botOn = !g_botOn;
    if (g_botOn) StartMode(); else g_mode = M_IDLE;
    g_target = 0; g_lapFrames = 0; g_visited = 0;
}
static void OnTune(MenuEntry *) { g_tune = !g_tune; }
static void OnPol(MenuEntry *)
{
    if (!g_usePolicy && !LoadPolicy()) { MessageBox("policy.bin missing or wrong size")(); return; }
    g_usePolicy = !g_usePolicy;
}
static void OnForget(MenuEntry *)
{
    File::Remove(std::string(kDir) + LineName());
    File::Remove(std::string(kDir) + ParamName());
    LoadTrack();
    if (g_botOn) StartMode();
}
static void OnRec(MenuEntry *)
{
    g_rec = !g_rec;
    if (g_rec) { g_trail.clear(); g_mode = M_IDLE; }
    else { RebuildFromRaw(g_trail); SaveLine(); g_trail.clear(); }
}

namespace CTRPluginFramework {
void PatchProcess(FwkSettings &) {}
void OnProcessExit(void) {}
int main()
{
    SetupItems();
    g_rng ^= (u32)svcGetSystemTick();
    PluginMenu *m = new PluginMenu("MK7 Bot", 0, 3, 0, "Offline racing bot");
    m->SynchronizeWithFrame(true);
    m->Append(new MenuEntry("Bot on/off", nullptr, OnBot));
    m->Append(new MenuEntry("Self-tuning on/off", nullptr, OnTune));
    m->Append(new MenuEntry("Neural policy on/off", nullptr, OnPol));
    m->Append(new MenuEntry("Forget this track", nullptr, OnForget));
    m->Append(new MenuEntry("Manual line record (fallback)", nullptr, OnRec));
    m->Callback(BotFrame);
    OSD::Run(BotOverlay);
    m->Run();
    delete m;
    return 0;
}
}
